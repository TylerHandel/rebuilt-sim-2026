#!/usr/bin/env python3
"""Train the neural-net driver in the GPU simulator (tools/nn/gpusim.py): thousands of
simplified matches at once on the graphics card, 1v1 up to 3v3, roughly a hundred times more
matches per hour than the full game on the CPU.

  python tools/nn/train_gpu.py                 # train (Ctrl+C saves; --resume continues)
  python tools/nn/train_gpu.py --envs 8192     # more matches at once (more GPU memory)

The network sees and controls exactly what it does in the real game, so js/nn/driver.json works
there as is. The GPU simulator is simplified, so finish with some training in the real game:
  python tools/nn/train.py --resume --level 3  # same run folder: picks up this checkpoint

Teammates share the network (they learn to play together). Opponents: none, scripted bots,
older copies of itself (snapshots), and itself. Self-play turns on once it beats the bots. From
then on the reward shifts from points margin to winning (see reward()).
"""
import argparse
import base64
import collections
import copy
import json
import math
import os
import shutil
import signal
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, str(Path(__file__).parent))
from train import ROOT, ACT_CONT, ACT_BIN, Actor, mlp, export_policy, publish_file, upgrade_inputs  # noqa: E402
from gpusim import GpuSim, RS, OPP_NONE, OPP_BOT, OPP_SNAP, OPP_SELF  # noqa: E402

KIND = {OPP_NONE: 'alone', OPP_BOT: 'bot', OPP_SNAP: 'older', OPP_SELF: 'self'}


class TorchNorm:
    """Running observation mean / variance on the GPU (same state as train.py's RunningNorm)."""

    def __init__(self, n, dev):
        self.mean = torch.zeros(n, dtype=torch.float64, device=dev)
        self.var = torch.ones(n, dtype=torch.float64, device=dev)
        self.count = 1e-4

    def update(self, x):
        x = x.double()
        bm, bv, bc = x.mean(0), x.var(0, unbiased=False), x.shape[0]
        d = bm - self.mean
        tot = self.count + bc
        self.mean = self.mean + d * bc / tot
        self.var = (self.var * self.count + bv * bc + d * d * self.count * bc / tot) / tot
        self.count = tot

    @property
    def std(self):
        return torch.sqrt(self.var + 1e-8)

    def apply(self, x):
        return ((x - self.mean.float()) / self.std.float()).clamp(-10, 10)

    def state(self):
        return {'mean': self.mean.cpu().tolist(), 'var': self.var.cpu().tolist(), 'count': self.count}

    def load(self, s):
        self.mean = torch.tensor(s['mean'], dtype=torch.float64, device=self.mean.device)
        self.var = torch.tensor(s['var'], dtype=torch.float64, device=self.mean.device)
        self.count = s['count']

    def numpy(self):  # for export_policy
        class N:
            pass
        n = N()
        n.mean, n.std = self.mean.cpu().numpy(), self.std.cpu().numpy()
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
    return torch.cat([a_c, a_b], -1), lp_c + lp_b


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
        self.actor = Actor(self.D, a.hidden).to(self.dev)
        self.critic = mlp([self.D, *a.critic, 1]).to(self.dev)
        self.opt = torch.optim.Adam([*self.actor.parameters(), *self.critic.parameters()], lr=a.lr, eps=1e-5)
        self.norm = TorchNorm(self.D, self.dev)
        self.version = self.steps = self.updates = self.episodes = 0
        self.selfplay = False
        self.selfplay_step = 0
        self.pool = []
        self.snap_of = torch.zeros(self.N, dtype=torch.long, device=self.dev)
        self.hist = collections.defaultdict(lambda: collections.deque(maxlen=400))
        self.t_start = time.time()
        self.prev_elapsed = 0.0

    # ---- checkpoints (same format as train.py, so it can --resume there)
    def save(self):
        ck = {
            'actor': self.actor.state_dict(), 'critic': self.critic.state_dict(), 'opt': self.opt.state_dict(),
            'norm': self.norm.state(), 'version': self.version, 'steps': self.steps, 'updates': self.updates,
            'level': self.a.real_level, 'pool': [], 'obs_dim': self.D, 'hidden': self.a.hidden, 'critic_sizes': self.a.critic,
            'episodes': self.episodes, 'hist': {}, 'own_hist': {}, 'elapsed': self.elapsed(),
            'gpu': {'selfplay': self.selfplay, 'selfplay_step': self.selfplay_step, 'pool': [p.state_dict() for p in self.pool],
                    'hist': {k: list(v) for k, v in self.hist.items()}},
        }
        tmp = self.run / 'ckpt.tmp'
        torch.save(ck, tmp)
        os.replace(tmp, self.run / 'ckpt.pt')

    def load(self):
        if not (self.run / 'ckpt.pt').exists():
            sys.exit(f'Nothing to resume: {self.run / "ckpt.pt"} does not exist.\n'
                     f'Training progress is kept in the runs folder of the copy of the project you trained in. If you '
                     f'downloaded a new copy, copy the old runs folder (and js/nn/driver.json) into it, or leave out --resume to start fresh.')
        ck = torch.load(self.run / 'ckpt.pt', map_location=self.dev, weights_only=False)
        if ck['obs_dim'] > self.D:
            sys.exit(f'{self.run / "ckpt.pt"} was trained with a newer observation ({ck["obs_dim"]} > {self.D}); update the code')
        g = ck.get('gpu') or {}
        upgraded = upgrade_inputs(ck, self.D)
        if upgraded:
            for sd in g.get('pool', []):
                w = sd['body.0.weight']
                sd['body.0.weight'] = torch.cat([w, torch.zeros(w.shape[0], self.D - w.shape[1], dtype=w.dtype, device=w.device)], 1)
            print(f'Upgraded the network to the new observation ({self.D} inputs: teammates and all opponents for 3v3). '
                  f'It plays exactly as before until it learns to use them.')
        self.a.hidden, self.a.critic = ck['hidden'], ck['critic_sizes']
        self.actor = Actor(self.D, self.a.hidden).to(self.dev)
        self.critic = mlp([self.D, *self.a.critic, 1]).to(self.dev)
        self.actor.load_state_dict(ck['actor'])
        self.critic.load_state_dict(ck['critic'])
        self.opt = torch.optim.Adam([*self.actor.parameters(), *self.critic.parameters()], lr=self.a.lr, eps=1e-5)
        if ck.get('opt'):
            try:
                self.opt.load_state_dict(ck['opt'])
                for grp in self.opt.param_groups:
                    grp['lr'] = self.a.lr
            except ValueError:
                pass
        self.norm.load(ck['norm'])
        self.version, self.steps, self.updates, self.episodes = ck['version'], ck['steps'], ck['updates'], ck['episodes']
        self.selfplay = g.get('selfplay', False)
        self.selfplay_step = g.get('selfplay_step', self.steps if self.selfplay else 0)
        for sd in g.get('pool', []):
            p = copy.deepcopy(self.actor)
            p.load_state_dict(sd)
            self.pool.append(p.eval())
        for k, v in g.get('hist', {}).items():
            if k in ('bot', 'self', 'snapshot', 'none', 'alone', 'older'):
                continue  # older runs counted 1v1 only
            self.hist[k].extend(v)
        self.prev_elapsed = ck.get('elapsed', 0)
        print(f'Resumed {self.run.name}: {self.steps / 1e6:.1f}M decisions, {self.updates} updates' + (' (self-play on)' if self.selfplay else ''))

    def elapsed(self):
        return self.prev_elapsed + time.time() - self.t_start

    def publish(self):
        info = {'run': self.a.run, 'trainer': 'gpu', 'steps': self.steps, 'updates': self.updates, 'hours': round(self.elapsed() / 3600, 2),
                'date': time.strftime('%Y-%m-%d %H:%M'), 'device': str(self.dev), 'winWeight': round(self.win_weight(), 3),
                'vs': {k: round(float(np.mean(v)), 1) for k, v in self.hist.items() if v}}
        cur = self.run / 'policy' / 'current.json'
        export_policy(self.actor, self.norm.numpy(), cur, self.version, info)
        if self.a.publish:
            publish_file(cur, ROOT / 'js' / 'nn' / 'driver.json')

    # ---- opponents
    def set_opponents(self):
        if self.selfplay:
            probs = list(self.a.mix) if self.a.mix else [0.05, 0.25, 0.35, 0.35]
            if not self.pool:
                probs[3] += probs[2]
                probs[2] = 0.0
        else:
            probs = [0.2, 0.8, 0.0, 0.0]
        self.sim.opp_probs = torch.tensor(probs, device=self.dev, dtype=torch.float)

    def bot_winrate(self):
        h = [m for k, v in self.hist.items() if k.startswith('bot') for m in v]
        return (sum(m > 0 for m in h) / len(h), len(h)) if h else (0.0, 0)

    def check_selfplay(self):
        wr, n = self.bot_winrate()
        if not self.selfplay and n >= 200 and wr >= self.a.selfplay_at:
            self.selfplay = True
            self.selfplay_step = self.steps
            self.snapshot()
            self.set_opponents()
            print(f'\n*** It beats the scripted bots {wr:.0%} of the time: self-play on, and the reward starts shifting toward winning\n')

    def snapshot(self):
        p = copy.deepcopy(self.actor).eval()
        for q in p.parameters():
            q.requires_grad_(False)
        self.pool.append(p)
        if len(self.pool) > self.a.pool_size:
            self.pool.pop(0)

    # ---- reward: points margin early on, winning once it plays well
    def win_weight(self):
        if self.a.win_weight is not None:
            return self.a.win_weight
        if not self.selfplay:
            return 0.0
        return min(1.0, (self.steps - self.selfplay_step) / max(1.0, self.a.win_ramp))

    def reward(self, rew, done):
        """rew [...,5]: own alliance pts, their pts, FUEL this robot intaked, own inactive-HUB FUEL,
        margin after the step. A rising win weight w moves the reward from the points margin to:
          * a squashed margin, A*tanh(margin/S): closing a 10-point gap in a close match counts far
            more than adding 10 to a blowout,
          * a bonus of +-B at the final buzzer for winning or losing."""
        a = self.a
        w = self.win_weight()
        d_pts = rew[..., 0] - rew[..., 1]
        m_after = rew[..., 4]
        m_before = m_after - d_pts
        S, A, Bw = 40.0, 15.0, 10.0
        r = (1 - 0.7 * w) * d_pts / 10.0
        r = r + w * A * (torch.tanh(m_after / S) - torch.tanh(m_before / S))
        r = r + w * Bw * torch.sign(m_after) * done[:, None].float()
        k = max(0.0, 1.0 - self.steps / a.shaping_steps) if a.shaping_steps > 0 else 0.0
        return r + k * (0.03 * rew[..., 2] - 0.02 * rew[..., 3])

    # ---- live view for the dashboard
    def live(self, frames):
        if not frames:
            return
        self.live_seq += len(frames)
        data = {'seq': self.live_seq, 'dt': self.sim.dt, 'frames': frames[-60:], 'updates': self.updates,
                'robots': {k: [self.sim.P['robots'][k]['halfL'], self.sim.P['robots'][k]['halfW']] for k in self.sim.order}}
        for f in data['frames']:
            for e in f['envs']:
                if isinstance(e['balls'], (bytes, bytearray)):
                    e['balls'] = base64.b64encode(e['balls']).decode('ascii')
        tmp = self.run / 'live.tmp'
        tmp.write_text(json.dumps(data))
        try:
            os.replace(tmp, self.run / 'live.json')
        except PermissionError:
            pass

    # ---- training
    def train(self):
        a, sim, N, D, dev = self.a, self.sim, self.N, self.D, self.dev
        if a.resume:
            self.load()
        if a.selfplay and not self.selfplay:
            self.selfplay = True
            self.selfplay_step = self.steps
            self.snapshot()
            print('Self-play on (--selfplay)')
        self.set_opponents()
        sim.reset(torch.ones(N, dtype=torch.bool, device=dev))
        # spread the first matches out in time (each starts at a random point with a fresh field)
        sim.t = torch.rand(N, device=dev) * (sim.T_AUTO + sim.T_GAP + sim.T_TELE)
        sim.firstInactive = (torch.rand(N, device=dev) < 0.5).long()
        full = torch.zeros(N, dtype=torch.bool, device=dev)  # those first matches are partial: not counted
        print(f'Device: {dev}' + (f' ({torch.cuda.get_device_name(0)})' if dev.type == 'cuda' else '') +
              f'. {N} matches at once ({", ".join(f"{k}v{k} {p:.0%}" for k, p in zip((1, 2, 3), sim.team_probs.tolist()) if p > 0)}), {sim.sub} physics substeps per decision.')
        self.publish()
        T = a.rollout
        stop = {'flag': False}

        def on_sigint(*_):
            if stop['flag']:
                raise KeyboardInterrupt
            stop['flag'] = True
            print('\nStopping after this update (Ctrl+C again to quit right away) ...')

        signal.signal(signal.SIGINT, on_sigint)
        log = open(self.run / 'progress.jsonl', 'a')
        last_save = last_live = time.time()
        self.live_seq = 0
        live_envs = list(range(min(a.live, N)))
        frames = []
        ep = collections.defaultdict(list)
        try:
            while not stop['flag'] and self.steps < a.total_steps:
                t0 = time.time()
                obs_l, act_l, lp_l = [], [], []
                L_b = torch.zeros(T, N, RS, dtype=torch.bool, device=dev)
                V_b = torch.zeros(T, N, RS, device=dev)
                R_b = torch.zeros(T, N, RS, device=dev)
                done_b = torch.zeros(T, N, device=dev)
                valid_l = []
                obs = sim.obs()
                for t in range(T):
                    L = sim.learning()
                    x = self.norm.apply(obs)
                    act = sim.bot_actions()
                    with torch.no_grad():
                        xl = x[L]
                        al, lpl = sample(self.actor, xl)
                        act[L] = al
                        V_b[t][L] = self.critic(xl).squeeze(-1)
                        snapm = (sim.opp_kind == OPP_SNAP)[:, None] & sim.present & (sim.team == 1)
                        if snapm.any() and self.pool:
                            which = self.snap_of[:, None].expand(N, RS) % len(self.pool)
                            for i, p in enumerate(self.pool):
                                m = snapm & (which == i)
                                if m.any():
                                    act[m] = sample(p, x[m])[0]
                    auto, gap, tele, post, tt = sim.phase()
                    obs_l.append(obs[L])
                    act_l.append(al)
                    lp_l.append(lpl)
                    valid_l.append((auto | tele)[:, None].expand(N, RS)[L])
                    L_b[t] = L
                    rew, done = sim.step(act)
                    R_b[t] = self.reward(rew, done)
                    done_b[t] = done.float()
                    if live_envs:
                        frames.append({'w': round(time.time(), 3), 'envs': sim.frame(live_envs)})
                    if done.any():
                        idx = done.nonzero().squeeze(-1)
                        mA = sim.margin()[:, 0]
                        totA = sim.totals().gather(1, sim.alliance_idx()[:, :1]).squeeze(1)
                        for i in idx.tolist():
                            if not full[i]:
                                continue
                            key = f'{KIND[int(sim.opp_kind[i])]} {int(sim.k[i])}v{int(sim.k[i])}'
                            self.hist[key].append(float(mA[i]))
                            ep[key].append((float(totA[i]), float(mA[i])))
                            self.episodes += 1
                        sim.reset(done)
                        full |= done
                        if self.pool:
                            self.snap_of[done] = torch.randint(0, len(self.pool), (int(done.sum()),), device=dev)
                    obs = sim.obs()
                    if live_envs and time.time() - last_live > 1.0:
                        self.live(frames)
                        frames = []
                        last_live = time.time()
                sim_s = time.time() - t0
                # ---- advantages (GAE) per robot
                t1 = time.time()
                with torch.no_grad():
                    Lf = sim.learning()
                    Vn = torch.zeros(N, RS, device=dev)
                    Vn[Lf] = self.critic(self.norm.apply(obs[Lf])).squeeze(-1)
                    adv = torch.zeros(T, N, RS, device=dev)
                    last = torch.zeros(N, RS, device=dev)
                    for t in reversed(range(T)):
                        nv = Vn if t == T - 1 else V_b[t + 1]
                        nonterm = 1.0 - done_b[t][:, None]
                        delta = R_b[t] + a.gamma * nv * nonterm - V_b[t]
                        last = (delta + a.gamma * a.lam * nonterm * last) * L_b[t]
                        adv[t] = last
                    ret = adv + V_b
                    X = torch.cat(obs_l)
                    self.norm.update(X[torch.cat(valid_l)])
                    X = self.norm.apply(X)
                    ACT, LP = torch.cat(act_l), torch.cat(lp_l)
                    ADV = torch.cat([adv[t][L_b[t]] for t in range(T)])
                    RET = torch.cat([ret[t][L_b[t]] for t in range(T)])
                    VAL = torch.cat(valid_l).float()
                n = len(X)
                stats = collections.defaultdict(list)
                for _ in range(a.epochs):
                    perm = torch.randperm(n, device=dev)
                    for i in range(0, n, a.minibatch):
                        ix = perm[i:i + a.minibatch]
                        lp, ent, _, _ = self.actor.dist_terms(X[ix], ACT[ix])
                        ratio = (lp - LP[ix]).exp()
                        ad = ADV[ix]
                        ad = (ad - ad.mean()) / (ad.std() + 1e-8)
                        vm = VAL[ix]  # no policy gradient while the robot is disabled
                        pl = -(torch.min(ratio * ad, ratio.clamp(1 - a.clip, 1 + a.clip) * ad) * vm).sum() / vm.sum().clamp(min=1)
                        vl = 0.5 * ((self.critic(X[ix]).squeeze(-1) - RET[ix]) ** 2).mean()
                        loss = pl + a.vf * vl - a.ent * (ent * vm).sum() / vm.sum().clamp(min=1)
                        self.opt.zero_grad()
                        loss.backward()
                        nn.utils.clip_grad_norm_([*self.actor.parameters(), *self.critic.parameters()], 0.5)
                        self.opt.step()
                        with torch.no_grad():
                            stats['ent'].append(ent.mean().item())
                            stats['kl'].append((LP[ix] - lp).mean().item())
                upd_s = time.time() - t1
                self.version += 1
                self.updates += 1
                nsamp = int(VAL.sum().item())
                self.steps += nsamp
                self.check_selfplay()
                if self.selfplay and self.updates % a.snapshot_every == 0:
                    self.snapshot()
                if self.updates % a.publish_every == 0:
                    self.publish()
                sps = nsamp / max(1e-6, sim_s + upd_s)
                per_match = max(1.0, L_b.float().sum().item() / (T * N))  # learning robots per match
                mps = sps * 3600 / 1600 / per_match
                st = {k: float(np.mean(v)) for k, v in stats.items()}
                vs = '  '.join(f'{k}: {np.mean([m for _, m in v]):+.0f} ({sum(m > 0 for _, m in v)}/{len(v)})' for k, v in sorted(ep.items()))
                own_all = [o for v in ep.values() for o, _ in v]
                gpu = f'  gpu {torch.cuda.max_memory_allocated() / 2**30:.1f}GB' if dev.type == 'cuda' else ''
                print(f'[{self.elapsed() / 3600:5.2f}h] upd {self.updates:5d}  {self.steps / 1e6:8.2f}M decisions  {sps:7.0f}/s (~{mps:,.0f} matches/h)  '
                      f'pts {np.mean(own_all) if own_all else 0:5.1f}  win-weight {self.win_weight():.2f}  ent {st["ent"]:+.2f}  kl {st["kl"]:+.4f}  sim {sim_s:.1f}s upd {upd_s:.1f}s{gpu}')
                if vs:
                    print(f'      vs {vs}')
                elif self.updates % 10 == 0 and not full.all():
                    print(f'      (first matches still finishing: {full.float().mean().item():.0%} of the matches have started a full one)')
                if own_all:
                    rec = {'t': round(self.elapsed(), 1), 'steps': self.steps, 'updates': self.updates, 'level': int(self.selfplay), 'sps': round(sps, 1),
                           'own': round(float(np.mean(own_all)), 2), 'winWeight': round(self.win_weight(), 3), **{k: round(v, 5) for k, v in st.items()},
                           'vs': {k: round(float(np.mean([m for _, m in v])), 2) for k, v in ep.items()},
                           'win': {k: round(sum(m > 0 for _, m in v) / len(v), 3) for k, v in ep.items()}, 'trainer': 'gpu'}
                    log.write(json.dumps(rec) + '\n')
                    log.flush()
                    ep.clear()
                if time.time() - last_save > a.save_minutes * 60:
                    self.save()
                    last_save = time.time()
        finally:
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
    p.add_argument('--gamma', type=float, default=0.995)
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
    p.add_argument('--win-ramp', type=float, default=50e6, help='decisions after self-play starts over which the reward shifts to winning')
    p.add_argument('--win-weight', type=float, default=None, help='fix the win weight (0 = points margin only, 1 = mostly winning)')
    p.add_argument('--no-randomize', action='store_true', help="don't vary robot speed / intake / accuracy per match")
    p.add_argument('--live', type=int, default=16, help='matches streamed to the dashboard (0 = off)')
    p.add_argument('--snapshot-every', type=int, default=50)
    p.add_argument('--pool-size', type=int, default=8)
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
