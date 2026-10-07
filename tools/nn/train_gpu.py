#!/usr/bin/env python3
"""Train the neural-net driver in the GPU simulator (tools/nn/gpusim.py): thousands of
simplified matches at once on the graphics card, 1v1 up to 3v3, roughly a hundred times more
matches per hour than the full game on the CPU.

  python tools/nn/train_gpu.py                 # train (Ctrl+C saves; --resume continues)
  python tools/nn/train_gpu.py --envs 8192     # more matches at once (more GPU memory)

The network sees and controls exactly what it does in the real game, so js/nn/driver.json works
there as is. The GPU simulator is simplified, so finish with some training in the real game:
  python tools/nn/train.py --resume --level 3  # same run folder: picks up this checkpoint

Teammates share the network (they learn to play together), and the value network (the critic)
sees all three teammates' views, so it judges how the alliance as a whole is doing. Opponents:
none, scripted bots, older copies of itself (the ones it loses to more often), and itself.
Self-play turns on once it beats the bots. From then on the reward shifts from points margin to
winning (see reward()), and it looks further ahead (gamma rises).

Speed: a whole decision (the networks, the physics, the resets, the observations) is recorded
once as a CUDA graph and replayed, so the GPU never waits for Python; --compile also fuses the
simulator's math into a few big GPU kernels. Meanwhile the otherwise idle CPU plays the latest
network in the full game against the Champs AIs (--eval) for the dashboard's real-game record.
"""
import argparse
import base64
import collections
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, str(Path(__file__).parent))
from train import ROOT, ACT_CONT, ACT_BIN, Actor, mlp, export_policy, publish_file, upgrade_inputs, team_critic, critic_priv, load_demos  # noqa: E402
from gpusim import GpuSim, RS, OPP_NONE, OPP_BOT, OPP_SNAP, OPP_SELF  # noqa: E402

KIND = {OPP_NONE: 'alone', OPP_BOT: 'bot', OPP_SNAP: 'older', OPP_SELF: 'self'}
TEAM = 3  # views the team critic sees: the robot's own and its two teammates'


class TorchNorm:
    """Running observation mean / variance on the GPU (same state as train.py's RunningNorm).
    Updated in place, so a recorded CUDA graph always reads the current values."""

    def __init__(self, n, dev):
        self.mean = torch.zeros(n, dtype=torch.float64, device=dev)
        self.var = torch.ones(n, dtype=torch.float64, device=dev)
        self.m32 = torch.zeros(n, device=dev)
        self.s32 = torch.ones(n, device=dev)
        self.count = 1e-4
        self._sync()

    def _sync(self):
        self.m32.copy_(self.mean)
        self.s32.copy_(torch.sqrt(self.var + 1e-8))

    def update_moments(self, bm, bv, bc):
        d = bm - self.mean
        tot = self.count + bc
        self.var.copy_((self.var * self.count + bv * bc + d * d * self.count * bc / tot) / tot)
        self.mean.copy_(self.mean + d * bc / tot)
        self.count = tot
        self._sync()

    def apply(self, x):
        return ((x - self.m32) / self.s32).clamp(-10, 10)

    def state(self):
        return {'mean': self.mean.cpu().tolist(), 'var': self.var.cpu().tolist(), 'count': self.count}

    def load(self, s):
        self.mean.copy_(torch.tensor(s['mean'], dtype=torch.float64))
        self.var.copy_(torch.tensor(s['var'], dtype=torch.float64))
        self.count = s['count']
        self._sync()

    def numpy(self):  # for export_policy
        class N:
            pass
        n = N()
        n.mean, n.std = self.mean.cpu().numpy(), torch.sqrt(self.var + 1e-8).cpu().numpy()
        return n


def sample(actor, x):
    """Sample actions (and their log-probs) like js/nn/policy.js does in training."""
    mu, lg = actor(x)
    ls = actor.log_std.clamp(-3.0, 0.5)
    a_c = mu + ls.exp() * torch.randn_like(mu)
    p = torch.sigmoid(lg)
    a_b = (torch.rand_like(p) < p).float()
    lp_c = (-0.5 * ((a_c - mu) / ls.exp()) ** 2 - ls - 0.5 * math.log(2 * math.pi)).sum(-1)
    lp_b = -nn.functional.binary_cross_entropy_with_logits(lg, a_b, reduction='none').sum(-1)
    return torch.cat([a_c, a_b], -1), lp_c + lp_b, mu


class SnapBank:
    """The older copies of the network (the snapshot pool), stacked so they all run at once: each
    robot in a match against an older version gets the actions of the version that match picked."""

    def __init__(self, actor, n, dev):
        self.n = n
        lin = [m for m in actor.body if isinstance(m, nn.Linear)]
        z = lambda *s: torch.zeros(*s, device=dev)
        self.W = [z(n, m.out_features, m.in_features) for m in lin]
        self.b = [z(n, m.out_features) for m in lin]
        H = lin[-1].out_features
        self.Wmu, self.bmu = z(n, ACT_CONT, H), z(n, ACT_CONT)
        self.Wlg, self.blg = z(n, ACT_BIN, H), z(n, ACT_BIN)
        self.ls = z(n, ACT_CONT)

    @torch.no_grad()
    def set(self, i, sd):
        ks = sorted({k.split('.')[1] for k in sd if k.startswith('body.') and k.endswith('.weight')}, key=int)
        for j, k in enumerate(ks):
            self.W[j][i].copy_(sd[f'body.{k}.weight'])
            self.b[j][i].copy_(sd[f'body.{k}.bias'])
        self.Wmu[i].copy_(sd['mu.weight'])
        self.bmu[i].copy_(sd['mu.bias'])
        self.Wlg[i].copy_(sd['lg.weight'])
        self.blg[i].copy_(sd['lg.bias'])
        self.ls[i].copy_(sd['log_std'])

    def act(self, x, slot):
        """x [R,D] normalized observations, slot [R] which version -> sampled actions [R,7]."""
        n, R = self.n, x.shape[0]
        with torch.autocast(device_type=x.device.type, dtype=torch.bfloat16, enabled=x.device.type == 'cuda', cache_enabled=False):
            W0 = self.W[0].reshape(n * self.W[0].shape[1], -1)
            h = torch.tanh(nn.functional.linear(x, W0, self.b[0].reshape(-1)))       # [R, n*H0]: one big matmul
            h = h.view(R, n, -1).transpose(0, 1)                                     # [n,R,H0]
            for W, b in zip(self.W[1:], self.b[1:]):
                h = torch.tanh(torch.baddbmm(b[:, None], h, W.transpose(1, 2)))
            mu = torch.baddbmm(self.bmu[:, None], h, self.Wmu.transpose(1, 2)).float()
            lg = torch.baddbmm(self.blg[:, None], h, self.Wlg.transpose(1, 2)).float()
        sel = slot[None, :, None]
        mu = mu.gather(0, sel.expand(1, R, ACT_CONT))[0]
        lg = lg.gather(0, sel.expand(1, R, ACT_BIN))[0]
        ls = self.ls[slot].clamp(-3.0, 0.5)
        a_c = mu + ls.exp() * torch.randn_like(mu)
        a_b = (torch.rand_like(lg) < torch.sigmoid(lg)).float()
        return torch.cat([a_c, a_b], -1)


class Teacher:
    """Who the network learns from on its own matches (--teacher), on top of its own trial and
    error: the GPU simulator's scripted robot ('bot'), or an earlier network (a run folder, its
    ckpt.pt, or a policy .json such as js/nn/driver.json or a shared one). Each decision the
    teacher says what it would do in the very situation the network is in, so the network learns
    to recover from its own mistakes too (unlike copying recordings, which only show the
    teacher's own matches). Its pull fades out over --teacher-steps.
    act(raw) -> [R, 7]: drive / turn commands, then the probability of each button."""

    def __init__(self, spec, D, dev):
        self.kind = 'bot' if spec == 'bot' else 'net'
        self.name = spec
        if self.kind == 'bot':
            return
        path = Path(spec) if Path(spec).is_absolute() else ROOT / spec
        if path.is_dir():
            path = path / 'ckpt.pt'
        if not path.exists():
            sys.exit(f'--teacher: {path} not found (give a run folder, its ckpt.pt, a policy .json, or "bot")')
        if path.suffix == '.json':
            d = json.loads(path.read_text())
            hidden, old = d['hidden'], d['obsDim']
            actor = Actor(old, hidden)
            w = np.frombuffer(base64.b64decode(d['weights']), dtype='<f4')
            lins = [m for m in actor.body if isinstance(m, nn.Linear)] + [actor.mu, actor.lg]
            i = 0
            with torch.no_grad():
                for m in lins:
                    for t in (m.weight, m.bias):
                        t.copy_(torch.from_numpy(w[i:i + t.numel()].copy()).view_as(t))
                        i += t.numel()
            with torch.no_grad():
                actor.log_std.copy_(torch.tensor(d['logStd']))
            sd = actor.state_dict()
            mean, std = np.array(d['obsMean']), np.array(d['obsStd'])
        else:
            ck = torch.load(path, map_location='cpu', weights_only=False)
            hidden, old, sd = ck['hidden'], ck['obs_dim'], ck['actor']
            mean = np.array(ck['norm']['mean'])
            std = np.sqrt(np.array(ck['norm']['var']) + 1e-8)
        if old > D:
            sys.exit(f'--teacher {spec} was trained on a newer observation ({old} > {D}); update the code')
        if old < D:  # an older observation: the new inputs get zero weights (it plays as it did)
            sd = dict(sd)
            w0 = sd['body.0.weight']
            sd['body.0.weight'] = torch.cat([w0, torch.zeros(w0.shape[0], D - old, dtype=w0.dtype)], 1)
            mean = np.concatenate([mean, np.zeros(D - old)])
            std = np.concatenate([std, np.ones(D - old)])
        self.actor = Actor(D, hidden).to(dev)
        self.actor.load_state_dict(sd)
        self.actor.requires_grad_(False)
        self.mean = torch.tensor(mean, dtype=torch.float32, device=dev)
        self.std = torch.tensor(std, dtype=torch.float32, device=dev)
        self.hidden = hidden

    def act(self, raw):
        x = ((raw - self.mean) / self.std).clamp(-10, 10)
        mu, lg = self.actor(x)
        return torch.cat([mu, torch.sigmoid(lg)], -1)


class GpuTrainer:
    def __init__(self, a):
        self.a = a
        self.dev = torch.device(a.device if a.device != 'auto' else ('cuda' if torch.cuda.is_available() else 'cpu'))
        if self.dev.type == 'cuda':
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
        self.run = ROOT / 'runs' / a.run
        self.run.mkdir(parents=True, exist_ok=True)
        envs = a.envs or (4096 if self.dev.type == 'cuda' else 128)
        tp = [a.teams.count(k) for k in (1, 2, 3)] if a.teams else [0.25, 0.15, 0.6]
        self.sim = GpuSim(envs, device=self.dev, robots=a.robots, seed=a.seed, substeps=a.substeps, team_probs=tp, randomize=not a.no_randomize)
        self.N, self.D = self.sim.n, self.sim.D
        # NVIDIA speedups: bf16 tensor cores for the critic (the big network); fused kernels for the
        # simulator (torch.compile); the whole decision as a CUDA graph (configure())
        self.amp = a.amp and self.dev.type == 'cuda'
        if a.compile:
            print('torch.compile: fusing the simulator into GPU kernels (a few minutes the first time, faster later: it keeps a cache) ...')
            t = time.time()
            ok = self.sim.compile()
            if not ok and os.name == 'nt':
                print('  On Windows torch.compile needs Triton: .venv\\Scripts\\pip install triton-windows')
            print(f'  done in {time.time() - t:.0f} s')
        self.P = self.sim.PRIV if a.priv else 0   # privileged inputs the critic sees (gpusim.priv)
        self.actor = Actor(self.D, a.hidden).to(self.dev)
        self.critic = self._critic()
        self.opt = self._optimizer()
        self.norm = TorchNorm(self.D, self.dev)
        # PopArt: the critic predicts normalized returns; V() scales them back with these (changed in
        # place, so the recorded graph sees them). Starts as the identity.
        self.v_mu = torch.zeros((), device=self.dev)
        self.v_sig = torch.ones((), device=self.dev)
        self.v_init = False
        self.teacher = None
        self.version = self.steps = self.updates = self.episodes = 0
        self.selfplay = False
        self.selfplay_step = 0
        self.pool = []        # older versions {'sd', 'id', 'wins', 'games'}; pool[i] plays from bank slot i
        self.bank = SnapBank(self.actor, a.pool_size, self.dev)
        self.sim.set_snapshots(torch.zeros(a.pool_size))
        self.hist = collections.defaultdict(lambda: collections.deque(maxlen=400))
        self.t_start = time.time()
        self.prev_elapsed = 0.0
        # numbers the recorded graph reads: changed in place, never replaced
        self.w_t = torch.zeros((), device=self.dev)       # win weight
        self.k_t = torch.zeros((), device=self.dev)       # pickup-bonus weight
        self.t_idx = torch.zeros(1, dtype=torch.long, device=self.dev)
        self.graph = None
        self.evals = []

    def _critic(self):
        return mlp([TEAM * self.D + self.P, *self.a.critic, 1]).to(self.dev)

    def _optimizer(self):
        return torch.optim.Adam([*self.actor.parameters(), *self.critic.parameters()], lr=self.a.lr, eps=1e-5)

    def Vn(self, x):
        """Critic output: the normalized value (bf16 tensor cores with --amp)."""
        with torch.autocast(device_type='cuda', dtype=torch.bfloat16, enabled=self.amp, cache_enabled=False):
            return self.critic(x).squeeze(-1).float()

    def V(self, x):
        """Critic values in reward units."""
        return self.Vn(x) * self.v_sig + self.v_mu

    @torch.no_grad()
    def popart(self, ret):
        """Update the return statistics from this rollout's returns and rescale the critic's last
        layer so its values don't change (PopArt). Keeps the critic's targets near unit scale while
        the reward changes scale (the win bonus ramping in), and keeps its gradients from swamping
        the policy's in the shared gradient clip."""
        m, m2 = ret.mean().double(), (ret.double() ** 2).mean()
        b = 1.0 if not self.v_init else self.a.popart_beta
        mu0, sig0 = self.v_mu.clone(), self.v_sig.clone()
        nu0 = sig0.double() ** 2 + mu0.double() ** 2
        mu = (1 - b) * mu0.double() + b * m
        nu = (1 - b) * nu0 + b * m2
        sig = (nu - mu * mu).clamp(min=1e-4).sqrt().clamp(min=0.05)
        last = self.critic[-1]
        last.weight.mul_((sig0 / sig).float())
        last.bias.copy_(((sig0 * last.bias + mu0 - mu) / sig).float())
        self.v_mu.copy_(mu.float())
        self.v_sig.copy_(sig.float())
        self.v_init = True

    # ---- checkpoints (same format as train.py, so it can --resume there)
    def save(self):
        ck = {
            'actor': self.actor.state_dict(), 'critic': self.critic.state_dict(), 'opt': self.opt.state_dict(),
            'norm': self.norm.state(), 'version': self.version, 'steps': self.steps, 'updates': self.updates,
            'level': self.a.real_level, 'pool': [], 'obs_dim': self.D, 'hidden': self.a.hidden, 'critic_sizes': self.a.critic,
            'critic_in': 'team', 'priv_dim': self.P, 'vnorm': {'mu': float(self.v_mu), 'sig': float(self.v_sig)} if self.v_init else None,
            'episodes': self.episodes, 'hist': {}, 'own_hist': {}, 'elapsed': self.elapsed(),
            'gpu': {'selfplay': self.selfplay, 'selfplay_step': self.selfplay_step, 'pool': [p['sd'] for p in self.pool],
                    'pool_meta': [{k: p[k] for k in ('id', 'wins', 'games')} for p in self.pool], 'bc_start': getattr(self, 'bc_start', None),
                    'teacher_start': getattr(self, 'teacher_start', None), 'teacher': self.teacher.name if self.teacher else None,
                    'hist': {k: list(v) for k, v in self.hist.items()}},
        }
        tmp = self.run / 'ckpt.tmp'
        torch.save(ck, tmp)
        os.replace(tmp, self.run / 'ckpt.pt')

    def load(self):
        if not (self.run / 'ckpt.pt').exists():
            sys.exit(f'Nothing to resume: {self.run / "ckpt.pt"} does not exist.\n'
                     f'Training progress is kept in the runs folder of the copy of the project you trained in. If you '
                     f'downloaded a new copy, run import-old.bat in it (it copies the runs folder and the network over), '
                     f'or leave out --resume to start fresh.')
        ck = torch.load(self.run / 'ckpt.pt', map_location=self.dev, weights_only=False)
        if ck['obs_dim'] > self.D:
            sys.exit(f'{self.run / "ckpt.pt"} was trained with a newer observation ({ck["obs_dim"]} > {self.D}); update the code')
        g = ck.get('gpu') or {}
        old_dim = ck['obs_dim']
        if upgrade_inputs(ck, self.D):
            for sd in g.get('pool', []):
                w = sd['body.0.weight']
                sd['body.0.weight'] = torch.cat([w, torch.zeros(w.shape[0], self.D - old_dim, dtype=w.dtype, device=w.device)], 1)
            print(f'Upgraded the network to the new observation ({self.D} inputs: more robots to recognize). '
                  f'It plays exactly as before until it learns to use them.')
        if team_critic(ck):
            print("Upgraded the critic to see the teammates' views too (team critic): it judges exactly as before until it learns to use them.")
        if critic_priv(ck, self.P) and self.P:
            print('Upgraded the critic to see the whole match (privileged state: every robot\'s load, FUEL in flight, ...): it judges exactly as before until it learns to use it.')
        self.a.hidden, self.a.critic = ck['hidden'], ck['critic_sizes']
        self.actor = Actor(self.D, self.a.hidden).to(self.dev)
        self.critic = self._critic()
        self.actor.load_state_dict(ck['actor'])
        self.critic.load_state_dict(ck['critic'])
        self.opt = self._optimizer()
        if ck.get('opt'):
            try:
                self.opt.load_state_dict(ck['opt'])
                for grp in self.opt.param_groups:
                    grp['lr'] = self.a.lr
            except ValueError:
                pass
        self.norm.load(ck['norm'])
        v = ck.get('vnorm')
        if v:
            self.v_mu.fill_(v['mu'])
            self.v_sig.fill_(v['sig'])
            self.v_init = True
        self.version, self.steps, self.updates, self.episodes = ck['version'], ck['steps'], ck['updates'], ck['episodes']
        self.selfplay = g.get('selfplay', False)
        self.selfplay_step = g.get('selfplay_step', self.steps if self.selfplay else 0)
        self.bank = SnapBank(self.actor, self.a.pool_size, self.dev)
        sds = g.get('pool', [])
        meta = g.get('pool_meta') or [{'id': i} for i in range(len(sds))]
        for i, (sd, m) in enumerate(list(zip(sds, meta))[-self.a.pool_size:]):
            self.pool.append({'sd': sd, 'id': m.get('id', i), 'wins': m.get('wins', 0.0), 'games': m.get('games', 0.0)})
            self.bank.set(i, sd)
        for k, v in g.get('hist', {}).items():
            if k in ('bot', 'self', 'snapshot', 'none', 'alone', 'older'):
                continue  # older runs counted 1v1 only
            self.hist[k].extend(v)
        self.prev_elapsed = ck.get('elapsed', 0)
        self.saved_bc_start = g.get('bc_start')
        self.saved_teacher_start = g.get('teacher_start')
        self.saved_teacher = g.get('teacher')
        if self.a.teacher is None and g.get('teacher'):
            self.a.teacher = g['teacher']  # keeps learning from the teacher it started with (--teacher none: stop)
        print(f'Resumed {self.run.name}: {self.steps / 1e6:.1f}M decisions, {self.updates} updates' + (' (self-play on)' if self.selfplay else ''))

    def elapsed(self):
        return self.prev_elapsed + time.time() - self.t_start

    def publish(self):
        info = {'run': self.a.run, 'trainer': 'gpu', 'steps': self.steps, 'updates': self.updates, 'hours': round(self.elapsed() / 3600, 2),
                'date': time.strftime('%Y-%m-%d %H:%M'), 'device': str(self.dev), 'winWeight': round(self.win_weight(), 3), 'robots': self.a.robots,
                'vs': {k: round(float(np.mean(v)), 1) for k, v in self.hist.items() if v}}
        cur = self.run / 'policy' / 'current.json'
        export_policy(self.actor, self.norm.numpy(), cur, self.version, info)
        if self.a.publish:
            publish_file(cur, ROOT / 'js' / 'nn' / 'driver.json')

    # ---- opponents
    def wanted_mix(self):
        if self.selfplay:
            return list(self.a.mix) if self.a.mix else [0.05, 0.25, 0.35, 0.35]
        return [0.2, 0.8, 0.0, 0.0]

    def bot_winrate(self):
        h = [m for k, v in self.hist.items() if k.startswith('bot') for m in v]
        return (sum(m > 0 for m in h) / len(h), len(h)) if h else (0.0, 0)

    def check_selfplay(self):
        wr, n = self.bot_winrate()
        if not self.selfplay and n >= 200 and wr >= self.a.selfplay_at:
            self.selfplay = True
            self.selfplay_step = self.steps
            self.snapshot()
            print(f'\n*** It beats the scripted bots {wr:.0%} of the time: self-play on, and the reward starts shifting toward winning\n')
            self.configure()

    def snapshot(self):
        """Add the current network to the pool of older versions (replacing the oldest when full)."""
        sd = {k: v.detach().clone() for k, v in self.actor.state_dict().items()}
        e = {'sd': sd, 'id': self.version, 'wins': 0.0, 'games': 0.0}
        if len(self.pool) < self.a.pool_size:
            self.pool.append(e)
            i = len(self.pool) - 1
        else:
            i = self.evict()
            self.pool[i] = e
        self.bank.set(i, sd)
        self.update_pfsp()

    def evict(self):
        """Which older version makes room for a new one: of those it has played enough (keeping
        the 4 newest), the one it beats most easily. The versions that still give it trouble stay
        however old they are, so it can't forget how to beat them (no going round in circles);
        --pool-evict oldest drops the oldest instead."""
        ids = sorted(p['id'] for p in self.pool)
        newest = set(ids[-4:])
        cand = [j for j, p in enumerate(self.pool) if p['id'] not in newest and p['games'] >= 20]
        if self.a.pool_evict == 'oldest' or not cand:
            return min(range(len(self.pool)), key=lambda j: self.pool[j]['id'])
        return max(cand, key=lambda j: (self.pool[j]['wins'] + 1) / (self.pool[j]['games'] + 2))

    def update_pfsp(self):
        """Prioritized fictitious self-play: an older version it loses to gets played more often
        (one it always beats, rarely). Win rates count a draw as half, start at 50%."""
        p = torch.zeros(self.a.pool_size)
        for i, e in enumerate(self.pool):
            wr = (e['wins'] + 1) / (e['games'] + 2)
            p[i] = (1.0 - wr + 0.1) ** 2 if self.a.pfsp else 1.0
        self.sim.set_snapshots(p)

    # ---- reward: points margin early on, winning once it plays well
    def win_weight(self):
        if self.a.win_weight is not None:
            return self.a.win_weight
        if not self.selfplay:
            return 0.0
        return min(1.0, (self.steps - self.selfplay_step) / max(1.0, self.a.win_ramp))

    def shaping(self):
        a = self.a
        return max(0.0, 1.0 - self.steps / a.shaping_steps) if a.shaping_steps > 0 else 0.0

    def gamma(self):
        """Look further ahead once the reward is about winning the match: 0.995 -> 0.998 (about
        20 s -> 50 s of game time) as the win weight rises."""
        return self.a.gamma if self.a.gamma is not None else 0.995 + 0.003 * self.win_weight()

    def reward(self, rew, done):
        """rew [...,7]: own alliance pts, their pts, FUEL this robot intaked, own inactive-HUB FUEL,
        margin after the step, change of the drive / turn command, how hard it turns. Jerky
        driving costs a little (--smooth, --spin): the network has to hold a heading and a line
        instead of twitching back and forth. A rising win weight w moves the reward from the points margin to:
          * a squashed margin, A*tanh(margin/S): closing a 10-point gap in a close match counts far
            more than adding 10 to a blowout,
          * a bonus of +-B at the final buzzer for winning or losing.
        (w and the pickup-bonus weight are GPU numbers, so the recorded graph sees them change.)"""
        w, k = self.w_t, self.k_t
        d_pts = rew[..., 0] - rew[..., 1]
        m_after = rew[..., 4]
        m_before = m_after - d_pts
        S, A, Bw = 40.0, 15.0, 10.0
        r = (1 - 0.7 * w) * d_pts / 10.0
        r = r + w * A * (torch.tanh(m_after / S) - torch.tanh(m_before / S))
        r = r + w * Bw * torch.sign(m_after) * done.float()
        r = r - self.a.smooth * rew[..., 5] - self.a.spin * rew[..., 6]
        return r + k * (0.03 * rew[..., 2] - 0.02 * rew[..., 3])

    # ---- the fixed layout of a rollout
    @torch.no_grad()
    def configure(self):
        """Split the matches between the kinds of opponents, lay out which robots the network
        drives (fixed rows, so every decision has the same shapes), allocate the rollout buffers,
        start every match afresh at a random point in time, and record the decision as a graph."""
        a, sim, N, D, dev = self.a, self.sim, self.N, self.D, self.dev
        self.graph = None
        kr = sim.set_mix(self.wanted_mix())
        self.kind_np = sim.opp_kind.cpu().numpy()
        ar = lambda *x: torch.arange(*x, device=dev)
        s0, s1 = kr[OPP_SELF]
        L_env = torch.cat([ar(N).repeat_interleave(3), ar(s0, s1).repeat_interleave(3)])
        L_slot = torch.cat([ar(3).repeat(N), 3 + ar(3).repeat(s1 - s0)])
        self.L_env, self.L_flat = L_env, L_env * RS + L_slot
        self.NL = NL = len(L_env)
        # each row's two teammates: the rows of the other two slots of its alliance in that match
        rows = ar(NL)
        others = torch.tensor([[1, 2], [0, 2], [0, 1]], device=dev)[rows % 3]
        self.mate_row = (rows - rows % 3)[:, None] + others
        p0, p1 = kr[OPP_SNAP]
        self.S_env = ar(p0, p1).repeat_interleave(3) if p1 > p0 else None
        self.S_flat = self.S_env * RS + 3 + ar(3).repeat(p1 - p0) if p1 > p0 else None
        self.bots = kr[OPP_BOT] if kr[OPP_BOT][1] > kr[OPP_BOT][0] else None
        T = a.rollout
        self.buf = None
        if dev.type == 'cuda':
            torch.cuda.empty_cache()
        z = lambda *s, **k: torch.zeros(*s, device=dev, **k)
        self.buf = {
            'obs': z(T, NL, D, dtype=torch.float16), 'act': z(T, NL, ACT_CONT + ACT_BIN), 'lp': z(T, NL), 'v': z(T, NL), 'r': z(T, NL),
            'priv': z(T, NL, self.P, dtype=torch.float16), 'tact': z(T, NL, ACT_CONT + ACT_BIN) if self.teacher else None,
            'pres': z(T, NL, dtype=torch.bool), 'valid': z(T, NL, dtype=torch.bool),
            'done': z(T, N, dtype=torch.bool), 'margin': z(T, N), 'own': z(T, N), 'k': z(T, N, dtype=torch.long), 'slot': z(T, N, dtype=torch.long),
            'turn': z(T, NL), 'mstat': z(T, N, len(sim.MSTAT)),
        }
        nl = min(a.live, N)
        self.live_idx = torch.tensor([int((i + 0.5) * N / nl) for i in range(nl)], device=dev) if nl else None
        if nl:
            self.buf['live'] = z(T, nl, sim.live_pack(self.live_idx).shape[-1])
        self.update_pfsp()
        sim.reset(torch.ones(N, dtype=torch.bool, device=dev))
        # spread the matches out in time (each starts at a random point with a fresh field)
        sim.t.copy_(torch.rand(N, device=dev) * (sim.T_AUTO + sim.T_GAP + sim.T_TELE))
        sim.firstInactive.copy_((torch.rand(N, device=dev) < 0.5).long())
        self.full = np.zeros(N, dtype=bool)  # those first matches are partial: not counted
        self.obs_s = sim.obs().clone()
        self.priv_s = sim.priv().clone() if self.P else None
        if a.graph and dev.type == 'cuda':
            self.build_graph()
        parts = ', '.join(f'{KIND[k]} {hi - lo}' for k, (lo, hi) in enumerate(kr) if hi > lo)
        print(f'Matches against: {parts}. The network drives {NL} robots per decision' + (', recorded as a CUDA graph' if self.graph else '') + '.')

    def team_in(self, x, pres, priv=None):
        """Critic input: a robot's own view and its two teammates' (zeros for an absent one), then
        the whole match (privileged state, --priv)."""
        mr = self.mate_row.view(-1)
        m = x.index_select(0, mr).view(-1, 2, self.D) * pres.index_select(0, mr).view(-1, 2, 1)
        return torch.cat([x, m.view(-1, 2 * self.D)] + ([priv] if self.P else []), -1)

    @torch.no_grad()
    def decide(self):
        """One decision in every match, recorded into the rollout buffers at t_idx. Nothing here
        waits for the CPU (no reading values back, the same shapes every time), so it can be
        recorded as a CUDA graph."""
        sim, b, t, N, D = self.sim, self.buf, self.t_idx, self.N, self.D
        of = self.obs_s.view(N * RS, D)
        raw = of.index_select(0, self.L_flat).half()                     # stored like this, and what the network sees
        x = self.norm.apply(raw.float())
        pres = sim.present.view(-1).index_select(0, self.L_flat)
        pv = self.priv_s.view(N * RS, -1).index_select(0, self.L_flat) if self.P else None
        a_l, lp_l, mu_l = sample(self.actor, x)
        v_l = self.V(self.team_in(x, pres, pv))
        # the scripted robots' actions, for every robot (once: they keep some state), when they're
        # the teacher; otherwise only for the matches against them
        bot_all = sim.bot_actions() if self.teacher is not None and self.teacher.kind == 'bot' else None
        if self.teacher is not None:
            t_l = bot_all.view(N * RS, -1).index_select(0, self.L_flat) if bot_all is not None else self.teacher.act(raw.float())
        act = torch.zeros(N * RS, ACT_CONT + ACT_BIN, device=self.dev).index_copy(0, self.L_flat, a_l)
        if self.S_flat is not None:
            xs = self.norm.apply(of.index_select(0, self.S_flat))
            act = act.index_copy(0, self.S_flat, self.bank.act(xs, sim.snap_slot.index_select(0, self.S_env)))
        act = act.view(N, RS, -1)
        if self.bots:
            lo, hi = self.bots
            act[lo:hi, 3:] = (bot_all[lo:hi] if bot_all is not None else sim.bot_actions(slice(lo, hi)))[:, 3:]
        auto, gap, tele, post, tt = sim.phase()
        valid = (auto | tele).index_select(0, self.L_env) & pres
        rew, done = sim.step(act)
        r_l = self.reward(rew.view(N * RS, -1).index_select(0, self.L_flat), done.index_select(0, self.L_env))
        totA = sim.totals().gather(1, sim.alliance_idx()[:, :1]).squeeze(1)
        if self.P:
            b['priv'].index_copy_(0, t, pv.half().unsqueeze(0))
        if self.teacher is not None:
            b['tact'].index_copy_(0, t, t_l.unsqueeze(0))
        for k, v in (('obs', raw), ('act', a_l), ('lp', lp_l), ('v', v_l), ('r', r_l), ('pres', pres), ('valid', valid),
                     ('done', done), ('margin', rew[:, 0, 4]), ('own', totA), ('k', sim.k), ('slot', sim.snap_slot),
                     ('turn', mu_l[:, 2]), ('mstat', sim.match_stats())):
            b[k].index_copy_(0, t, v.unsqueeze(0))
        if self.live_idx is not None:
            b['live'].index_copy_(0, t, sim.live_pack(self.live_idx).unsqueeze(0))
        sim.reset(done)
        self.obs_s.copy_(sim.obs())
        if self.P:
            self.priv_s.copy_(sim.priv())
        t.add_(1)

    def build_graph(self):
        """Record decide() as a CUDA graph. If anything about it fails, train without one."""
        sim = self.sim
        try:
            s = torch.cuda.Stream()
            s.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(s):
                self.t_idx.zero_()
                self.decide()  # first run: compiles / sets up whatever it needs
                torch.cuda.set_sync_debug_mode('error')  # a step that waits for the CPU can't be recorded: find out now
                try:
                    for _ in range(2):
                        self.t_idx.zero_()
                        self.decide()
                finally:
                    torch.cuda.set_sync_debug_mode('default')
            torch.cuda.current_stream().wait_stream(s)
            torch.cuda.synchronize()
            g = torch.cuda.CUDAGraph()
            self.t_idx.zero_()
            with torch.cuda.graph(g):
                self.decide()
            sim._rebind()
            # check it: a few replays must keep the matches sane
            self.t_idx.zero_()
            for _ in range(3):
                g.replay()
            torch.cuda.synchronize()
            if torch.isnan(self.obs_s).any().item() or (sim.bst[:, :sim.B] > 4).any().item() or not torch.isfinite(sim.pos).all().item():
                raise RuntimeError('the recorded graph produced invalid matches')
            self.graph = g
        except Exception as e:  # noqa: BLE001 - train without the graph (a little slower)
            torch.cuda.synchronize()
            sim._rebind()
            self.graph = None
            print(f'CUDA graph not used ({str(e).splitlines()[0][:160]}); training without it')
        self.t_idx.zero_()

    # ---- live view for the dashboard
    def live(self, frames):
        """Add frames to the last couple hundred and write them out for the dashboard."""
        if not frames:
            return
        for f in frames:
            for e in f['envs']:
                e['balls'] = base64.b64encode(e['balls']).decode('ascii')
        self.live_q.extend(frames)
        self.live_seq += len(frames)
        data = {'seq': self.live_seq, 'dt': self.sim.dt, 'frames': list(self.live_q), 'updates': self.updates,
                'robots': {k: [self.sim.P['robots'][k]['halfL'], self.sim.P['robots'][k]['halfW']] for k in self.sim.order}}
        tmp = self.run / 'live.tmp'
        tmp.write_text(json.dumps(data))
        try:
            os.replace(tmp, self.run / 'live.json')
        except PermissionError:
            pass

    # ---- real-game scoreboard on the CPU
    def start_eval(self):
        """Full-game matches of the latest published network against the Champs AIs, on the CPU
        (tools/nn/eval.mjs, low priority): what the dashboard's real-game record shows."""
        if self.a.eval <= 0 or not self.a.publish:
            return
        node = shutil.which('node')
        if not node or not (ROOT / 'node_modules' / '@dimforge').exists():
            print('Real-game scoreboard off: it needs Node.js and the packages (nn-setup.bat). --eval 0 hides this.')
            return
        out = self.run / 'eval.jsonl'
        hook = (ROOT / 'tools' / 'node-env.mjs').as_uri()
        flags = {'creationflags': subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.BELOW_NORMAL_PRIORITY_CLASS} if os.name == 'nt' else {'start_new_session': True}
        for i in range(self.a.eval):
            teams = '3,1' if self.a.eval == 1 else '3' if i % 2 == 0 else '1'
            cmd = [node, '--import', hook, str(ROOT / 'tools' / 'nn' / 'eval.mjs'), '--out', str(out), '--teams', teams,
                   '--seed', str(i + self.updates), '--parent', str(os.getpid())] + (['--robots', ','.join(self.a.robots)] if self.a.robots else [])
            try:
                p = subprocess.Popen(cmd, cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=open(self.run / f'eval{i}.log', 'w'), **flags)
            except OSError as e:
                print(f'Real-game scoreboard not started ({e}). --eval 0 hides this.')
                return
            if os.name != 'nt':
                try:
                    os.setpriority(os.PRIO_PROCESS, p.pid, 10)
                except (AttributeError, OSError):
                    pass
            self.evals.append(p)
        print(f'Real-game scoreboard: {len(self.evals)} full-game matches at a time on the CPU vs the Champs AIs, on the dashboard (--eval 0 turns it off)')

    def stop_eval(self):
        for p in self.evals:
            try:
                p.terminate()
            except OSError:
                pass

    # ---- learning from recorded driving (the pre-programmed AIs, or yours)
    def load_bc(self):
        """--bc FOLDER: recordings to imitate (tools/nn/record-ai.mjs, or the game's recordings).
        Kept on the CPU in half precision; minibatches go to the GPU."""
        a = self.a
        print(f'Loading recorded driving from {a.bc} ...')
        d = load_demos(self.bc_dir, self.D, max_n=a.bc_max, half=True) if self.bc_dir.exists() else None
        self.bc_files = self.bc_count()
        if d is None:
            if a.record_ai > 0:
                print('  none yet: the AIs are being recorded in the background; it starts imitating them once some matches are in')
                return False
            sys.exit(f'No recordings found in {a.bc}. Record the AIs first: nn-record-ai.bat (or add --record-ai 10)')
        first = self.bc_obs is None
        self.bc_obs = torch.from_numpy(np.ascontiguousarray(d[0]))
        self.bc_act = torch.from_numpy(np.ascontiguousarray(d[1]))
        if first:
            # continuing a run that was already imitating: keep fading from where it was
            self.bc_start = self.saved_bc_start if getattr(self, 'saved_bc_start', None) is not None and self.a.resume else self.steps
        return True

    def bc_count(self):
        return len(list(self.bc_dir.glob('*.aidemo'))) + len(list(self.bc_dir.glob('*.json'))) if self.bc_dir.exists() else 0

    def start_record(self):
        """--record-ai N: record the pre-programmed AIs on N CPU threads while it trains (low
        priority), into the --bc folder, up to --bc-max decisions."""
        a = self.a
        node = shutil.which('node')
        if a.record_ai <= 0 or not a.bc or not node:
            return
        flags = {'creationflags': subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.BELOW_NORMAL_PRIORITY_CLASS} if os.name == 'nt' else {'start_new_session': True}
        cmd = [node, '--import', (ROOT / 'tools' / 'node-env.mjs').as_uri(), str(ROOT / 'tools' / 'nn' / 'record-ai.mjs'), '--out', str(self.bc_dir),
               '--samples', str(a.bc_max), '--workers', str(a.record_ai), '--parent', str(os.getpid())] + (['--robots', ','.join(a.robots)] if a.robots else [])
        try:
            p = subprocess.Popen(cmd, cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=open(self.run / 'record-ai.log', 'w'), **flags)
        except OSError as e:
            print(f'Recording the AIs not started ({e})')
            return
        if os.name != 'nt':
            try:
                os.setpriority(os.PRIO_PROCESS, p.pid, 10)
            except (AttributeError, OSError):
                pass
        self.evals.append(p)  # stopped with the scoreboard
        print(f'Recording the pre-programmed AIs on {a.record_ai} CPU threads into {a.bc} (up to {a.bc_max:,} decisions); it picks up new recordings every {a.bc_reload:g} min')

    def bc_batch(self, n):
        idx = torch.randint(0, len(self.bc_obs), (n,))
        x = self.bc_obs[idx].to(self.dev, non_blocking=True).float()
        return self.norm.apply(x), self.bc_act[idx].to(self.dev, non_blocking=True)

    def bc_loss(self, x, y):
        mu, lg = self.actor(x)
        return ((mu - y[:, :ACT_CONT]) ** 2).sum(-1).mean() + nn.functional.binary_cross_entropy_with_logits(lg, y[:, ACT_CONT:], reduction='none').sum(-1).mean()

    def bc_weight(self):
        if self.bc_obs is None or self.a.bc_steps <= 0:
            return 0.0
        return self.a.bc_weight * max(0.0, 1.0 - (self.steps - self.bc_start) / self.a.bc_steps)

    def teacher_weight(self):
        """(weight of the teacher's pull, warming up?): during the first --teacher-warmup updates
        of a teacher it only copies it; then the pull fades out over --teacher-steps decisions."""
        if self.teacher is None:
            return 0.0, False
        a = self.a
        warm = self.updates < self.teacher_upd0 + a.teacher_warmup
        if warm:
            return a.teacher_weight, True
        return a.teacher_weight * max(0.0, 1.0 - (self.steps - self.teacher_start) / max(1.0, a.teacher_steps)), False

    def bc_pretrain(self):
        """A fresh network first copies the recorded driving (supervised), so training starts from
        how the AIs play instead of from random twitching."""
        a, X = self.a, self.bc_obs
        with torch.no_grad():  # observation statistics from the recordings
            s1 = torch.zeros(self.D, dtype=torch.float64, device=self.dev)
            s2 = torch.zeros(self.D, dtype=torch.float64, device=self.dev)
            for i in range(0, len(X), 65536):
                xo = X[i:i + 65536].to(self.dev).double()
                s1 += xo.sum(0)
                s2 += (xo * xo).sum(0)
            mean = s1 / len(X)
            self.norm.update_moments(mean, (s2 / len(X) - mean * mean).clamp(min=0), float(len(X)))
        opt = torch.optim.Adam(self.actor.parameters(), lr=1e-3)
        steps = max(1, int(a.bc_epochs * len(X) / 4096))
        print(f'Copying the recorded driving: {len(X):,} decisions, {a.bc_epochs} passes ...')
        tot = 0.0
        for i in range(steps):
            loss = self.bc_loss(*self.bc_batch(4096))
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += loss.item()
            if (i + 1) % max(1, steps // 10) == 0:
                print(f'  {100 * (i + 1) / steps:3.0f}%  imitation loss {tot / max(1, steps // 10):.4f}')
                tot = 0.0
        with torch.no_grad():
            self.actor.log_std.fill_(-1.2)  # it starts out driving like them, with a little exploration
        self.publish()

    # ---- training
    def train(self):
        a, sim, N, D, dev = self.a, self.sim, self.N, self.D, self.dev
        if a.resume:
            self.load()
        self.bc_obs = None
        if a.bc:
            self.bc_dir = Path(a.bc) if Path(a.bc).is_absolute() else ROOT / a.bc
            if self.load_bc() and (not a.resume or a.bc_pretrain):
                self.bc_pretrain()
            print(f'It keeps imitating the recordings a little (weight {a.bc_weight}) while it trains, fading out over {a.bc_steps / 1e6:.0f}M decisions.')
        if a.teacher == 'none':
            a.teacher = None
        if a.teacher:
            self.teacher = Teacher(a.teacher, D, dev)
            resumed = a.resume and getattr(self, 'saved_teacher_start', None) is not None and getattr(self, 'saved_teacher', None) == a.teacher
            self.teacher_start = self.saved_teacher_start if resumed else self.steps
            self.teacher_upd0 = -10**9 if resumed else self.updates  # warm-up only when a teacher is new
            if self.teacher.kind == 'net' and not a.resume and not a.bc:
                # a new network learning from a trained one: start from its observation statistics
                self.norm.load({'mean': self.teacher.mean.double().cpu().tolist(), 'var': (self.teacher.std.double().cpu() ** 2).tolist(), 'count': 1e6})
                with torch.no_grad():  # and explore as much as it does
                    self.actor.log_std.copy_(self.teacher.actor.log_std)
            who = "the GPU simulator's scripted robot" if self.teacher.kind == 'bot' else f'{a.teacher} ({"x".join(map(str, self.teacher.hidden))} network)'
            print(f'Teacher: {who}. ' + (f'It copies it for {a.teacher_warmup} updates, then ' if a.teacher_warmup and not resumed else 'It ') +
                  f'keeps learning from it (weight {a.teacher_weight}) on its own matches, fading out over {a.teacher_steps / 1e6:.0f}M decisions.')
        self.selfplay_pending = bool(a.selfplay and not self.selfplay and self.teacher_weight()[1])
        if a.selfplay and not self.selfplay and not self.selfplay_pending:
            self.selfplay = True
            self.selfplay_step = self.steps
            self.snapshot()
            print('Self-play on (--selfplay)')
        tp = (sim.team_probs / sim.team_probs.sum()).tolist()
        print(f'Device: {dev}' + (f' ({torch.cuda.get_device_name(0)})' if dev.type == 'cuda' else '') +
              f'. {N} matches at once ({", ".join(f"{k}v{k} {p:.0%}" for k, p in zip((1, 2, 3), tp) if p > 0)}), {sim.sub} physics substeps per decision.')
        self.configure()
        self.publish()
        self.start_eval()
        self.start_record()
        last_bc = time.time()
        T = a.rollout
        stop = {'flag': False}

        def on_sigint(*_):
            if stop['flag']:
                raise KeyboardInterrupt
            stop['flag'] = True
            print('\nStopping after this update (Ctrl+C again to quit right away) ...')

        signal.signal(signal.SIGINT, on_sigint)
        log = open(self.run / 'progress.jsonl', 'a')
        last_save = time.time()
        self.live_seq = 0
        self.live_q = collections.deque(maxlen=96)
        ep = collections.defaultdict(list)
        robot_st = []
        by_robot = collections.defaultdict(lambda: [0, 0.0, 0.0, 0.0, 0.0, 0.0])  # matches, wins, margin, own pts, FUEL, shots
        try:
            while not stop['flag'] and self.steps < a.total_steps:
                t0 = time.time()
                self.w_t.fill_(self.win_weight())
                self.k_t.fill_(self.shaping())
                self.t_idx.zero_()
                for _ in range(T):
                    if self.graph is not None:
                        self.graph.replay()
                    else:
                        self.decide()
                b = self.buf
                res = torch.stack([b['done'].float(), b['margin'], b['own'], b['k'].float(), b['slot'].float()]).cpu().numpy()
                mst = b['mstat'].cpu().numpy()
                # how it turns (its intended turn, as the game sees it): how hard, and how often it flips direction
                with torch.no_grad():
                    mw, vd = b['turn'], b['valid']
                    flip = (mw[1:].sign() != mw[:-1].sign()) & (mw[1:].abs() > 0.3) & (mw[:-1].abs() > 0.3) & vd[1:] & vd[:-1]
                    turn_stats = torch.stack([(mw.abs().clamp(max=1) * vd).sum() / vd.sum().clamp(min=1), flip.sum() / (vd[1:] & vd[:-1]).sum().clamp(min=1)]).tolist()
                live = b['live'].cpu().numpy() if self.live_idx is not None else None
                t1 = time.time()
                sim_s = t1 - t0
                # ---- finished matches
                n_done = 0
                for t in range(T):
                    for e in np.nonzero(res[0, t])[0]:
                        n_done += 1
                        if not self.full[e]:
                            self.full[e] = True
                            continue
                        kind, k, m = int(self.kind_np[e]), int(res[3, t, e]), float(res[1, t, e])
                        key = f'{KIND[kind]} {k}v{k}'
                        self.hist[key].append(m)
                        ep[key].append((float(res[2, t, e]), m))
                        robot_st.append(mst[t, e])
                        if kind != OPP_NONE:
                            r = by_robot[sim.order[int(mst[t, e, -1])]]
                            r[0] += 1
                            r[1] += 1.0 if m > 0 else 0.5 if m == 0 else 0.0
                            r[2] += m
                            r[3] += float(res[2, t, e])
                            r[4] += float(mst[t, e, 0])
                            r[5] += float(mst[t, e, 1])
                        self.episodes += 1
                        s = int(res[4, t, e])
                        if kind == OPP_SNAP and s < len(self.pool):
                            self.pool[s]['games'] += 1
                            self.pool[s]['wins'] += 1.0 if m > 0 else 0.5 if m == 0 else 0.0
                if live is not None:
                    self.live([dict(f, w=round(t0 + (i + 1) * (t1 - t0) / T, 3)) for i, f in enumerate(sim.live_frames(live))])
                # ---- advantages (GAE) per robot
                t1 = time.time()
                gamma, lam, NL = self.gamma(), a.lam, self.NL
                with torch.no_grad():
                    pres_now = sim.present.view(-1).index_select(0, self.L_flat)
                    x_now = self.norm.apply(self.obs_s.view(N * RS, D).index_select(0, self.L_flat).half().float())
                    pv_now = self.priv_s.view(N * RS, -1).index_select(0, self.L_flat).half().float() if self.P else None
                    Vn = self.V(self.team_in(x_now, pres_now, pv_now))
                    P_b = b['pres'].float()
                    D_b = b['done'].float().index_select(1, self.L_env)
                    adv = torch.zeros(T, NL, device=dev)
                    last = torch.zeros(NL, device=dev)
                    for t in reversed(range(T)):
                        nv = Vn if t == T - 1 else b['v'][t + 1]
                        nonterm = 1.0 - D_b[t]
                        delta = b['r'][t] + gamma * nv * nonterm - b['v'][t]
                        last = (delta + gamma * lam * nonterm * last) * P_b[t]
                        adv[t] = last
                    ret = (adv + b['v']).view(-1)
                    adv = adv.view(-1)
                    rows = b['pres'].view(-1).nonzero().squeeze(-1)
                    # how much of the returns the critic explains (1 = all, 0 = no better than the mean)
                    ev = (1 - (ret[rows] - b['v'].view(-1)[rows]).var() / ret[rows].var().clamp(min=1e-8)).item()
                    if a.popart:
                        self.popart(ret[rows])
                # ---- PPO, a minibatch at a time (observations stay fp16 until then)
                OBS, PRES = b['obs'].view(T * NL, D), b['pres'].view(-1)
                ACT, LP, VAL = b['act'].view(T * NL, -1), b['lp'].view(-1), b['valid'].view(-1).float()
                PRIV = b['priv'].view(T * NL, -1) if self.P else None
                TACT = b['tact'].view(T * NL, -1) if self.teacher is not None else None
                n = len(rows)
                ent_sum = torch.zeros((), device=dev)
                bc_sum = torch.zeros((), device=dev)
                t_sum = torch.zeros((), device=dev)
                pl_sum = torch.zeros((), device=dev)
                vl_sum = torch.zeros((), device=dev)
                bc_w = self.bc_weight()
                t_w, warm = self.teacher_weight()
                kl_sum = torch.zeros((), device=dev)
                nmb = 0
                epochs_done = 0
                for _ in range(a.epochs):
                    perm = rows[torch.randperm(n, device=dev)]
                    kl_ep = torch.zeros((), device=dev)
                    nmb_ep = 0
                    for i in range(0, n, a.minibatch):
                        ix = perm[i:i + a.minibatch]
                        x = self.norm.apply(OBS[ix].float())
                        mf = (ix - ix % NL)[:, None] + self.mate_row[ix % NL]
                        xm = self.norm.apply(OBS[mf.view(-1)].float()).view(-1, 2, D) * PRES[mf].float()[..., None]
                        lp, ent, mu_s, lg_s = self.actor.dist_terms(x, ACT[ix])
                        ratio = (lp - LP[ix]).exp()
                        ad = adv[ix]
                        ad = (ad - ad.mean()) / (ad.std() + 1e-8)
                        vm = VAL[ix]  # no policy gradient while the robot is disabled
                        vs = vm.sum().clamp(min=1)
                        pl = -(torch.min(ratio * ad, ratio.clamp(1 - a.clip, 1 + a.clip) * ad) * vm).sum() / vs
                        cin = torch.cat([x, xm.view(-1, 2 * D)] + ([PRIV[ix].float()] if self.P else []), -1)
                        # the critic learns normalized returns (PopArt; with --no-popart the scale stays 1)
                        vl = 0.5 * ((self.Vn(cin) - (ret[ix] - self.v_mu) / self.v_sig) ** 2).mean()
                        if warm:  # learning to play like the teacher first: no trial and error yet
                            loss = a.vf * vl
                        else:
                            loss = pl + a.vf * vl - a.ent * (ent * vm).sum() / vs
                        if t_w > 0:
                            ta = TACT[ix]
                            tl = ((mu_s - ta[:, :ACT_CONT]) ** 2).sum(-1) + nn.functional.binary_cross_entropy_with_logits(lg_s, ta[:, ACT_CONT:], reduction='none').sum(-1)
                            tl = (tl * vm).sum() / vs
                            loss = loss + t_w * tl
                            with torch.no_grad():
                                t_sum += tl
                        if bc_w > 0:
                            bl = self.bc_loss(*self.bc_batch(4096))
                            loss = loss + bc_w * bl
                            with torch.no_grad():
                                bc_sum += bl
                        self.opt.zero_grad()
                        loss.backward()
                        nn.utils.clip_grad_norm_([*self.actor.parameters(), *self.critic.parameters()], 0.5)
                        self.opt.step()
                        with torch.no_grad():
                            ent_sum += ent.mean()
                            kl_ep += (LP[ix] - lp).mean()
                            pl_sum += pl
                            vl_sum += vl
                        nmb += 1
                        nmb_ep += 1
                    kl_sum += kl_ep
                    epochs_done += 1
                    # it has moved far enough from the network that played the rollout: stop here
                    # (more epochs on the same data would only overfit it, and cost time)
                    if a.target_kl > 0 and not warm and (kl_ep / max(1, nmb_ep)).item() > a.target_kl:
                        break
                # ---- observation statistics: after the update, which used what the rollout saw
                with torch.no_grad():
                    s1 = torch.zeros(D, dtype=torch.float64, device=dev)
                    s2 = torch.zeros(D, dtype=torch.float64, device=dev)
                    cnt = torch.zeros((), dtype=torch.float64, device=dev)
                    for t in range(T):
                        m = b['valid'][t].double()[:, None]
                        xo = b['obs'][t].double()
                        s1 += (xo * m).sum(0)
                        s2 += (xo * xo * m).sum(0)
                        cnt += m.sum()
                    c = cnt.item()
                    if c > 0:
                        mean = s1 / c
                        self.norm.update_moments(mean, (s2 / c - mean * mean).clamp(min=0), c)
                upd_s = time.time() - t1
                self.version += 1
                self.updates += 1
                nsamp = int(VAL.sum().item())
                self.steps += nsamp
                for e in self.pool:  # forget old results slowly: the network keeps changing
                    e['wins'] *= 0.99
                    e['games'] *= 0.99
                if self.teacher is not None and self.teacher_weight() == (0.0, False):
                    print(f'\nIt has finished learning from its teacher ({self.teacher.name}): on its own from here\n')
                    self.teacher = None
                    self.configure()
                if self.selfplay_pending and not self.teacher_weight()[1]:
                    # it has copied its teacher: now its first version joins the older versions
                    self.selfplay_pending = False
                    self.selfplay = True
                    self.selfplay_step = self.steps
                    self.snapshot()
                    print('\nIt has copied its teacher: self-play on (--selfplay)\n')
                    self.configure()
                self.check_selfplay()
                if self.selfplay and self.updates % a.snapshot_every == 0:
                    self.snapshot()
                elif self.pool:
                    self.update_pfsp()
                if self.updates % a.publish_every == 0:
                    self.publish()
                sps = nsamp / max(1e-6, sim_s + upd_s)
                mph = n_done * 3600 / max(1e-6, sim_s + upd_s)
                ent_m, kl_m = (ent_sum / max(1, nmb)).item(), (kl_sum / max(1, nmb)).item()
                imit = f'  imitation {(bc_sum / max(1, nmb)).item():.3f} (x{bc_w:.2f})' if bc_w > 0 else ''
                if t_w > 0:
                    imit += f'  teacher {(t_sum / max(1, nmb)).item():.3f} (x{t_w:.2f}{", copying it first" if warm else ""})'
                vs = '  '.join(f'{k}: {np.mean([m for _, m in v]):+.0f} ({sum(m > 0 for _, m in v)}/{len(v)})' for k, v in sorted(ep.items()))
                own_all = [o for v in ep.values() for o, _ in v]
                gpu = f'  gpu {torch.cuda.max_memory_allocated() / 2**30:.1f}GB' if dev.type == 'cuda' else ''
                print(f'[{self.elapsed() / 3600:5.2f}h] upd {self.updates:5d}  {self.steps / 1e6:8.2f}M decisions  {sps:7.0f}/s (~{mph:,.0f} matches/h)  '
                      f'pts {np.mean(own_all) if own_all else 0:5.1f}  turn flips {turn_stats[1] * 600:4.0f}/min  win-weight {self.win_weight():.2f}  gamma {gamma:.4f}  ent {ent_m:+.2f}  kl {kl_m:+.4f}  '
                      f'critic {ev:+.2f}  epochs {epochs_done}  '
                      f'sim {sim_s:.1f}s upd {upd_s:.1f}s{gpu}{imit}')
                if vs:
                    print(f'      vs {vs}')
                elif self.updates % 10 == 0 and not self.full.all():
                    print(f'      (first matches still finishing: {self.full.mean():.0%} of the matches have started a full one)')
                if self.pool and self.updates % 25 == 0:
                    pp = (sim.pfsp_p / sim.pfsp_p.sum().clamp(min=1e-9)).tolist()
                    print('      older versions (its win rate against each / share of those matches): ' + '  '.join(
                        f'v{e["id"]} {(e["wins"] + 1) / (e["games"] + 2):.0%}/{pp[i]:.0%}' for i, e in enumerate(self.pool)))
                # every update: the dashboard shows speed and learning from the start, match results once matches finish
                rec = {'t': round(self.elapsed(), 1), 'steps': self.steps, 'updates': self.updates, 'level': int(self.selfplay), 'sps': round(sps, 1),
                       'own': round(float(np.mean(own_all)), 2) if own_all else None, 'winWeight': round(self.win_weight(), 3), 'gamma': round(gamma, 4),
                       'ent': round(ent_m, 5), 'kl': round(kl_m, 5), 'ev': round(ev, 3), 'epochs': epochs_done,
                       'vs': {k: round(float(np.mean([m for _, m in v])), 2) for k, v in ep.items()},
                       'win': {k: round(sum(m > 0 for _, m in v) / len(v), 3) for k, v in ep.items()}, 'trainer': 'gpu'}
                rec['n'] = {k: len(v) for k, v in ep.items()}
                if robot_st:
                    ms = np.mean(np.stack(robot_st), 0)
                    rec['sim'] = {'intaked': round(float(ms[0]), 1), 'shots': round(float(ms[1]), 1),
                                  'turn': round(turn_stats[0], 3), 'flipsPerMin': round(turn_stats[1] * 600, 1)}
                    # per match: the network's robot (play style) and its alliance's score
                    play = sim.T_AUTO + sim.T_TELE
                    rec['match'] = {'intaked': round(float(ms[0]), 2), 'shots': round(float(ms[1]), 2), 'passes': round(float(ms[2]), 2),
                                    'dist': round(float(ms[3]), 1), 'own': round(float(ms[4] / play), 4), 'neutral': round(float(ms[5] / play), 4),
                                    'opp': round(float(ms[6] / play), 4), 'load': round(float(ms[7] / play), 2), 'auto': round(float(ms[8]), 2),
                                    'tele': round(float(ms[9]), 2), 'wasted': round(float(ms[10]), 2), 'oppPts': round(float(ms[11]), 2)}
                if by_robot:
                    rec['robots'] = {k: [v[0], v[1], round(v[2], 1), round(v[3], 1), round(v[4], 1), round(v[5], 1)] for k, v in by_robot.items()}
                # how it learns: losses, exploration noise of the drive / turn commands, pulls toward a
                # teacher or recordings, where the time goes, and the older versions it plays
                rec['loss'] = {'policy': round((pl_sum / max(1, nmb)).item(), 5), 'value': round((vl_sum / max(1, nmb)).item(), 5)}
                if t_w > 0:
                    rec['loss']['teacher'] = round((t_sum / max(1, nmb)).item(), 5)
                if bc_w > 0:
                    rec['loss']['imitation'] = round((bc_sum / max(1, nmb)).item(), 5)
                rec['pull'] = {'teacher': round(t_w, 4), 'imitation': round(bc_w, 4)}
                rec['std'] = [round(v, 4) for v in self.actor.log_std.detach().clamp(-3.0, 0.5).exp().tolist()]
                rec['time'] = {'sim': round(sim_s, 2), 'upd': round(upd_s, 2)}
                rec['matchesPerHour'] = round(mph)
                rec['wall'] = round(time.time())
                if dev.type == 'cuda':
                    rec['gpuGB'] = round(torch.cuda.max_memory_allocated() / 2**30, 2)
                if self.pool:
                    pp = (sim.pfsp_p / sim.pfsp_p.sum().clamp(min=1e-9)).tolist()
                    rec['pool'] = [[e['id'], round((e['wins'] + 1) / (e['games'] + 2), 3), round(pp[i], 3), round(e['games'], 1)] for i, e in enumerate(self.pool)]
                rec['version'] = self.version
                log.write(json.dumps(rec) + '\n')
                log.flush()
                ep.clear()
                robot_st.clear()
                by_robot.clear()
                if a.bc and time.time() - last_bc > a.bc_reload * 60:
                    last_bc = time.time()
                    if self.bc_count() > self.bc_files and self.load_bc():
                        print(f'      now imitating {len(self.bc_obs):,} recorded AI decisions')
                if time.time() - last_save > a.save_minutes * 60:
                    self.save()
                    last_save = time.time()
        finally:
            self.stop_eval()
            print('Saving ...')
            self.save()
            self.publish()
            log.close()
            print(f'Saved {self.run / "ckpt.pt"}' + (' and js/nn/driver.json' if a.publish else ''))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--run', default='driver', help='run name (folder under runs/); train.py --resume continues the same run in the real game')
    p.add_argument('--resume', action='store_true')
    p.add_argument('--device', default='auto')
    p.add_argument('--envs', type=int, default=0, help='matches at once (default 4096 on a GPU)')
    p.add_argument('--teams', type=int, nargs='+', choices=[1, 2, 3], default=None,
                   help='match sizes to play, repeat to weight them: --teams 3 (only 3v3), --teams 1 3 3 (1/3 1v1, 2/3 3v3). Default: 25%% 1v1, 15%% 2v2, 60%% 3v3')
    p.add_argument('--substeps', type=int, default=4, help='physics substeps per decision: 2 is faster and coarser')
    p.add_argument('--rollout', type=int, default=32, help='decisions per match between updates')
    p.add_argument('--minibatch', type=int, default=16384)
    p.add_argument('--epochs', type=int, default=4)
    p.add_argument('--lr', type=float, default=3e-4)
    p.add_argument('--gamma', type=float, default=None, help='discount (default: 0.995, rising to 0.998 as the reward shifts to winning)')
    p.add_argument('--lam', type=float, default=0.95)
    p.add_argument('--clip', type=float, default=0.2)
    p.add_argument('--ent', type=float, default=0.003)
    p.add_argument('--vf', type=float, default=0.5)
    p.add_argument('--hidden', type=int, nargs='+', default=[512, 256])
    p.add_argument('--critic', type=int, nargs='+', default=[1024, 512, 256])
    p.add_argument('--shaping-steps', type=float, default=200e6, help='decisions over which the pickup bonus fades out')
    p.add_argument('--robots', nargs='+', default=None, help='robots it learns to drive (default: all)')
    p.add_argument('--selfplay-at', type=float, default=0.6, help='win rate against the bots that turns on self-play')
    p.add_argument('--selfplay', action='store_true', help='turn self-play on now (without waiting to beat the bots)')
    p.add_argument('--mix', type=float, nargs=4, metavar=('ALONE', 'BOT', 'OLDER', 'ITSELF'),
                   help='share of self-play matches alone / vs the bots / vs older versions / vs itself (default 0.05 0.25 0.35 0.35)')
    p.add_argument('--pfsp', action=argparse.BooleanOptionalAction, default=True,
                   help='play the older versions it loses to more often (--no-pfsp: all equally)')
    p.add_argument('--pool-size', type=int, default=16, help='older versions kept to play against')
    p.add_argument('--snapshot-every', type=int, default=50, help='updates between adding the current network to the older versions')
    p.add_argument('--win-ramp', type=float, default=50e6, help='decisions after self-play starts over which the reward shifts to winning')
    p.add_argument('--win-weight', type=float, default=None, help='fix the win weight (0 = points margin only, 1 = mostly winning)')
    p.add_argument('--bc', help='folder of recorded driving to learn from: the pre-programmed AIs (nn-record-ai.bat writes recordings-ai) or yours')
    p.add_argument('--bc-weight', type=float, default=0.5, help='how much it keeps imitating the recordings while it trains ...')
    p.add_argument('--bc-steps', type=float, default=300e6, help='... fading out over this many decisions')
    p.add_argument('--bc-epochs', type=float, default=3, help='passes over the recordings when a new network first copies them')
    p.add_argument('--bc-pretrain', action='store_true', help='copy the recordings first even when resuming (overwrites much of what it learned)')
    p.add_argument('--record-ai', type=int, default=0, help='record the pre-programmed AIs on this many CPU threads while it trains, into the --bc folder')
    p.add_argument('--bc-reload', type=float, default=20, help='minutes between picking up new recordings')
    p.add_argument('--bc-max', type=int, default=2000000, help='use at most this many recorded decisions (a random sample)')
    p.add_argument('--teacher', help="learn from a teacher on its own matches: 'bot' (the GPU simulator's scripted robot), a run folder "
                   "(runs/driver), its ckpt.pt, or a policy .json. With a new --run and a bigger --hidden this grows a bigger network from a trained one")
    p.add_argument('--teacher-weight', type=float, default=1.0, help="how strongly it's pulled toward the teacher ...")
    p.add_argument('--teacher-steps', type=float, default=200e6, help='... fading out over this many decisions')
    p.add_argument('--teacher-warmup', type=int, default=30, help='updates at the start of a new teacher where it only copies it (no trial and error yet)')
    p.add_argument('--popart', action=argparse.BooleanOptionalAction, default=True, help='normalize the returns the critic learns (PopArt; --no-popart to turn off)')
    p.add_argument('--popart-beta', type=float, default=0.05, help='how fast the return statistics follow the latest rollouts')
    p.add_argument('--priv', action=argparse.BooleanOptionalAction, default=True, help="the critic sees the whole match (every robot's load, FUEL in flight, ...); --no-priv to turn off")
    p.add_argument('--target-kl', type=float, default=0.02, help='stop the update epochs early once the network has moved this far from the one that played (0 = always all epochs)')
    p.add_argument('--pool-evict', choices=['easiest', 'oldest'], default='easiest', help='which older version a new one replaces when the pool is full')
    p.add_argument('--smooth', type=float, default=0.005, help='cost of changing the drive / turn command between decisions (stops twitchy driving)')
    p.add_argument('--spin', type=float, default=0.003, help='cost of turning (it should turn when it needs to, not all the time)')
    p.add_argument('--no-randomize', action='store_true', help="don't vary robot speed / intake / accuracy per match")
    p.add_argument('--live', type=int, default=4, help='matches streamed to the dashboard (0 = off)')
    p.add_argument('--eval', type=int, default=2, help='full-game matches at a time on the CPU for the real-game scoreboard (0 = off)')
    p.add_argument('--amp', action=argparse.BooleanOptionalAction, default=True, help='bf16 tensor cores for the critic (--no-amp to turn off)')
    p.add_argument('--compile', action='store_true', help='fuse the simulator into compiled GPU kernels (torch.compile; needs Triton)')
    p.add_argument('--graph', action=argparse.BooleanOptionalAction, default=True, help='record each decision as a CUDA graph (--no-graph to turn off)')
    p.add_argument('--publish-every', type=int, default=10)
    p.add_argument('--save-minutes', type=float, default=10)
    p.add_argument('--total-steps', type=float, default=float('inf'))
    p.add_argument('--real-level', type=int, default=3, help='opponent level train.py starts at when it resumes this run')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--no-publish', dest='publish', action='store_false')
    a = p.parse_args()
    GpuTrainer(a).train()


if __name__ == '__main__':
    main()
