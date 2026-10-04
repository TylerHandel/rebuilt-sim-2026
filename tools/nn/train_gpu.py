#!/usr/bin/env python3
"""Train the neural-net driver in the GPU simulator (tools/nn/gpusim.py): thousands of
simplified matches at once on the graphics card, roughly a hundred times more matches per hour
than the full game on the CPU.

  python tools/nn/train_gpu.py                 # train (Ctrl+C saves; --resume continues)
  python tools/nn/train_gpu.py --envs 8192     # more matches at once (more GPU memory)

The network sees and controls exactly what it does in the real game, so js/nn/driver.json works
there as is. The GPU simulator is simplified, so finish with a little training in the real game:
  python tools/nn/train.py --resume --level 3  # same run folder: picks up this checkpoint

Opponents: none, a scripted Scorer bot, older copies of itself (snapshots), and itself (both
robots learn). Self-play turns on once it beats the bot.
"""
import argparse
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
from train import ROOT, OBS_VERSION, ACT_CONT, ACT_BIN, Actor, mlp, export_policy  # noqa: E402
from gpusim import GpuSim, OPP_NONE, OPP_BOT, OPP_SNAP, OPP_SELF  # noqa: E402


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
    act = torch.cat([a_c, a_b], -1)
    lp_c = (-0.5 * ((a_c - mu) / ls.exp()) ** 2 - ls - 0.5 * math.log(2 * math.pi)).sum(-1)
    lp_b = -nn.functional.binary_cross_entropy_with_logits(lg, a_b, reduction='none').sum(-1)
    return act, lp_c + lp_b


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
        self.sim = GpuSim(envs, device=self.dev, robots=a.robots, seed=a.seed, substeps=a.substeps)
        self.N, self.D = self.sim.n, self.sim.D
        self.actor = Actor(self.D, a.hidden).to(self.dev)
        self.critic = mlp([self.D, *a.critic, 1]).to(self.dev)
        self.opt = torch.optim.Adam([*self.actor.parameters(), *self.critic.parameters()], lr=a.lr, eps=1e-5)
        self.norm = TorchNorm(self.D, self.dev)
        self.version = self.steps = self.updates = self.episodes = 0
        self.selfplay = False
        self.pool = []           # snapshot actors (on the GPU)
        self.snap_of = torch.zeros(self.N, dtype=torch.long, device=self.dev)
        self.hist = collections.defaultdict(lambda: collections.deque(maxlen=400))
        self.own_hist = collections.defaultdict(lambda: collections.deque(maxlen=400))
        self.t_start = time.time()
        self.prev_elapsed = 0.0

    # ---- checkpoints (same format as train.py, so it can --resume there)
    def save(self):
        ck = {
            'actor': self.actor.state_dict(), 'critic': self.critic.state_dict(), 'opt': self.opt.state_dict(),
            'norm': self.norm.state(), 'version': self.version, 'steps': self.steps, 'updates': self.updates,
            'level': self.a.real_level, 'pool': [], 'obs_dim': self.D, 'hidden': self.a.hidden, 'critic_sizes': self.a.critic,
            'episodes': self.episodes, 'hist': {}, 'own_hist': {}, 'elapsed': self.elapsed(),
            'gpu': {'selfplay': self.selfplay, 'pool': [p.state_dict() for p in self.pool],
                    'hist': {k: list(v) for k, v in self.hist.items()}, 'own_hist': {k: list(v) for k, v in self.own_hist.items()}},
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
        if ck['obs_dim'] != self.D:
            sys.exit(f'{self.run / "ckpt.pt"} was trained with a different observation ({ck["obs_dim"]} vs {self.D}); start a new --run')
        self.a.hidden, self.a.critic = ck['hidden'], ck['critic_sizes']
        self.actor = Actor(self.D, self.a.hidden).to(self.dev)
        self.critic = mlp([self.D, *self.a.critic, 1]).to(self.dev)
        self.actor.load_state_dict(ck['actor'])
        self.critic.load_state_dict(ck['critic'])
        self.opt = torch.optim.Adam([*self.actor.parameters(), *self.critic.parameters()], lr=self.a.lr, eps=1e-5)
        try:
            self.opt.load_state_dict(ck['opt'])
            for g in self.opt.param_groups:
                g['lr'] = self.a.lr
        except ValueError:
            pass
        self.norm.load(ck['norm'])
        self.version, self.steps, self.updates, self.episodes = ck['version'], ck['steps'], ck['updates'], ck['episodes']
        g = ck.get('gpu') or {}
        self.selfplay = g.get('selfplay', False)
        for sd in g.get('pool', []):
            p = copy.deepcopy(self.actor)
            p.load_state_dict(sd)
            self.pool.append(p.eval())
        for k, v in g.get('hist', {}).items():
            self.hist[k].extend(v)
        for k, v in g.get('own_hist', {}).items():
            self.own_hist[k].extend(v)
        self.prev_elapsed = ck.get('elapsed', 0)
        print(f'Resumed {self.run.name}: {self.steps / 1e6:.1f}M decisions, {self.updates} updates' + (' (self-play on)' if self.selfplay else ''))

    def elapsed(self):
        return self.prev_elapsed + time.time() - self.t_start

    def publish(self):
        info = {'run': self.a.run, 'trainer': 'gpu', 'steps': self.steps, 'updates': self.updates, 'hours': round(self.elapsed() / 3600, 2),
                'date': time.strftime('%Y-%m-%d %H:%M'), 'device': str(self.dev),
                'vs': {k: round(float(np.mean(v)), 1) for k, v in self.hist.items() if v}}
        cur = self.run / 'policy' / 'current.json'
        export_policy(self.actor, self.norm.numpy(), cur, self.version, info)
        if self.a.publish:
            shutil.copyfile(cur, ROOT / 'js' / 'nn' / 'driver.json')

    # ---- opponents
    def set_opponents(self):
        if self.selfplay:
            probs = list(self.a.mix) if self.a.mix else [0.05, 0.25, 0.35, 0.35]
            if not self.pool:  # no snapshots yet: play the current version instead
                probs[3] += probs[2]
                probs[2] = 0.0
        else:
            probs = [0.25, 0.75, 0.0, 0.0]
        self.sim.opp_probs = torch.tensor(probs, device=self.dev)

    def check_selfplay(self):
        h = self.hist['bot']
        if not self.selfplay and len(h) >= 200 and sum(m > 0 for m in h) / len(h) >= self.a.selfplay_at:
            self.selfplay = True
            self.snapshot()
            self.set_opponents()
            print(f'\n*** It beats the scripted bot {sum(m > 0 for m in h) / len(h):.0%} of the time: self-play on\n')

    def snapshot(self):
        p = copy.deepcopy(self.actor).eval()
        for q in p.parameters():
            q.requires_grad_(False)
        self.pool.append(p)
        if len(self.pool) > self.a.pool_size:
            self.pool.pop(0)

    def opponent_actions(self, obs1, x1):
        """Actions for robot 1 by opponent kind; x1 = normalized obs. Returns act, logp (self envs)."""
        sim = self.sim
        act = sim.bot_actions(1)
        kind = sim.opp_kind
        lp = torch.zeros(self.N, device=self.dev)
        snap = kind == OPP_SNAP
        if snap.any() and self.pool:
            for i, p in enumerate(self.pool):
                m = snap & (self.snap_of % len(self.pool) == i)
                if m.any():
                    with torch.no_grad():
                        a_, _ = sample(p, x1[m])
                    act[m] = a_
        selfm = kind == OPP_SELF
        if selfm.any():
            with torch.no_grad():
                a_, l_ = sample(self.actor, x1[selfm])
            act[selfm] = a_
            lp[selfm] = l_
        return act, lp

    # ---- training
    def reward(self, rew):
        a = self.a
        r = (rew[..., 0] - rew[..., 1]) / 10.0
        k = max(0.0, 1.0 - self.steps / a.shaping_steps) if a.shaping_steps > 0 else 0.0
        return r + k * (0.03 * rew[..., 2] - 0.02 * rew[..., 3])

    def train(self):
        a, sim, N, D, dev = self.a, self.sim, self.N, self.D, self.dev
        if a.resume:
            self.load()
        if a.selfplay and not self.selfplay:
            self.selfplay = True
            self.snapshot()
            print('Self-play on (--selfplay)')
        self.set_opponents()
        sim.reset(torch.ones(N, dtype=torch.bool, device=dev))
        # spread the first matches out in time (each starts at a random point of the match with a
        # fresh field), so later updates always see every part of the match at once
        sim.t = torch.rand(N, device=dev) * (sim.T_AUTO + sim.T_GAP + sim.T_TELE)
        full = torch.zeros(N, dtype=torch.bool, device=dev)  # those first matches are partial: not counted
        sim.firstInactive = (torch.rand(N, device=dev) < 0.5).long()
        print(f'Device: {dev}' + (f' ({torch.cuda.get_device_name(0)})' if dev.type == 'cuda' else '') + f'. {N} matches at once, {sim.sub} physics substeps per decision.')
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
        last_save = time.time()
        ep_own = collections.defaultdict(list)
        kind_names = {OPP_NONE: 'none', OPP_BOT: 'bot', OPP_SNAP: 'snapshot', OPP_SELF: 'self'}
        try:
            while not stop['flag'] and self.steps < a.total_steps:
                t0 = time.time()
                obs_b = torch.zeros(T, N, 2, D, device=dev)
                act_b = torch.zeros(T, N, 2, ACT_CONT + ACT_BIN, device=dev)
                lp_b = torch.zeros(T, N, 2, device=dev)
                rew_b = torch.zeros(T, N, 2, device=dev)
                done_b = torch.zeros(T, N, device=dev)
                valid_b = torch.zeros(T, N, 2, dtype=torch.bool, device=dev)
                train_b = torch.zeros(T, N, 2, dtype=torch.bool, device=dev)
                obs = sim.obs()
                for t in range(T):
                    x = self.norm.apply(obs)
                    with torch.no_grad():
                        a0, l0 = sample(self.actor, x[:, 0])
                        a1, l1 = self.opponent_actions(obs[:, 1], x[:, 1])
                    auto, gap, tele, post, tt = sim.phase()
                    en = (auto | tele)
                    obs_b[t] = obs
                    act_b[t, :, 0], act_b[t, :, 1] = a0, a1
                    lp_b[t, :, 0], lp_b[t, :, 1] = l0, l1
                    valid_b[t] = en[:, None] & sim.present
                    train_b[t, :, 0] = True
                    train_b[t, :, 1] = sim.opp_kind == OPP_SELF
                    rew, done = sim.step(torch.stack([a0, a1], 1))
                    rew_b[t] = self.reward(rew)
                    done_b[t] = done.float()
                    if done.any():
                        idx = done.nonzero().squeeze(-1)
                        tot = sim.totals()
                        al = sim.alliance_idx()
                        own = tot.gather(1, al[:, :1]).squeeze(1)
                        oth = tot.gather(1, (1 - al[:, :1])).squeeze(1)
                        for i in idx.tolist():
                            if not full[i]:
                                continue
                            k = kind_names[int(sim.opp_kind[i])]
                            self.hist[k].append(float(own[i] - oth[i]))
                            self.own_hist[k].append(float(own[i]))
                            ep_own[k].append((float(own[i]), float(own[i] - oth[i])))
                            self.episodes += 1
                        sim.reset(done)
                        full |= done
                        if self.pool:
                            self.snap_of[done] = torch.randint(0, len(self.pool), (int(done.sum()),), device=dev)
                    obs = sim.obs()
                sim_s = time.time() - t0
                # ---- advantages (GAE) for every robot slot that trains
                t1 = time.time()
                with torch.no_grad():
                    self.norm.update(obs_b[:, :, 0][valid_b[:, :, 0]])
                    X = self.norm.apply(obs_b)
                    V = self.critic(X).squeeze(-1)                                # [T,N,2]
                    Vn = self.critic(self.norm.apply(obs)).squeeze(-1)           # [N,2]
                    adv = torch.zeros_like(V)
                    last = torch.zeros(N, 2, device=dev)
                    for t in reversed(range(T)):
                        nv = Vn if t == T - 1 else V[t + 1]
                        nonterm = 1.0 - done_b[t][:, None]
                        delta = rew_b[t] + a.gamma * nv * nonterm - V[t]
                        last = delta + a.gamma * a.lam * nonterm * last
                        adv[t] = last
                    ret = adv + V
                use = train_b
                Xf, Af, LPf = X[use], act_b[use], lp_b[use]
                ADVf, RETf, VALf = adv[use], ret[use], valid_b[use]
                n = len(Xf)
                stats = collections.defaultdict(list)
                for _ in range(a.epochs):
                    perm = torch.randperm(n, device=dev)
                    for i in range(0, n, a.minibatch):
                        ix = perm[i:i + a.minibatch]
                        lp, ent, _, _ = self.actor.dist_terms(Xf[ix], Af[ix])
                        ratio = (lp - LPf[ix]).exp()
                        ad = ADVf[ix]
                        ad = (ad - ad.mean()) / (ad.std() + 1e-8)
                        vm = VALf[ix].float()  # no policy gradient while the robot is disabled
                        pl = -(torch.min(ratio * ad, ratio.clamp(1 - a.clip, 1 + a.clip) * ad) * vm).sum() / vm.sum().clamp(min=1)
                        vl = 0.5 * ((self.critic(Xf[ix]).squeeze(-1) - RETf[ix]) ** 2).mean()
                        loss = pl + a.vf * vl - a.ent * (ent * vm).sum() / vm.sum().clamp(min=1)
                        self.opt.zero_grad()
                        loss.backward()
                        nn.utils.clip_grad_norm_([*self.actor.parameters(), *self.critic.parameters()], 0.5)
                        self.opt.step()
                        with torch.no_grad():
                            stats['ent'].append(ent.mean().item())
                            stats['kl'].append((LPf[ix] - lp).mean().item())
                            stats['v'].append(vl.item())
                upd_s = time.time() - t1
                self.version += 1
                self.updates += 1
                self.steps += int(valid_b[:, :, 0].sum().item())
                self.check_selfplay()
                if self.selfplay and self.updates % a.snapshot_every == 0:
                    self.snapshot()
                if self.updates % a.publish_every == 0:
                    self.publish()
                sps = valid_b[:, :, 0].sum().item() / max(1e-6, sim_s + upd_s)
                st = {k: float(np.mean(v)) for k, v in stats.items()}
                vs = '  '.join(f'{k} {np.mean([m for _, m in v]):+.0f} ({sum(m > 0 for _, m in v)}/{len(v)})' for k, v in sorted(ep_own.items()))
                own_all = [o for v in ep_own.values() for o, _ in v]
                gpu = f'  gpu {torch.cuda.max_memory_allocated() / 2**30:.1f}GB' if dev.type == 'cuda' else ''
                print(f'[{self.elapsed() / 3600:5.2f}h] upd {self.updates:5d}  {self.steps / 1e6:8.2f}M decisions  {sps:7.0f}/s ({sps * 3600 / 1600:,.0f} matches/h)  '
                      f'pts {np.mean(own_all) if own_all else 0:5.1f}  ent {st["ent"]:+.2f}  kl {st["kl"]:+.4f}  sim {sim_s:.1f}s upd {upd_s:.1f}s{gpu}')
                if vs:
                    print(f'      vs: {vs}')
                rec = {'t': round(self.elapsed(), 1), 'steps': self.steps, 'updates': self.updates, 'level': int(self.selfplay), 'sps': round(sps, 1),
                       'own': round(float(np.mean(own_all)) if own_all else 0, 2), **{k: round(v, 5) for k, v in st.items()},
                       'vs': {k: round(float(np.mean([m for _, m in v])), 2) for k, v in ep_own.items()},
                       'win': {k: round(sum(m > 0 for _, m in v) / len(v), 3) for k, v in ep_own.items()}, 'trainer': 'gpu'}
                if not own_all and self.updates % 10 == 0 and not full.all():
                    print(f'      (first matches are still finishing: {full.float().mean().item():.0%} of the envs have started a full match)')
                if own_all:
                    log.write(json.dumps(rec) + '\n')
                    log.flush()
                    ep_own.clear()
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
    p.add_argument('--selfplay-at', type=float, default=0.6, help='win rate against the bot that turns on self-play')
    p.add_argument('--selfplay', action='store_true', help='turn self-play on now (without waiting to beat the bot)')
    p.add_argument('--mix', type=float, nargs=4, metavar=('ALONE', 'BOT', 'OLDER', 'ITSELF'),
                   help='share of self-play matches alone / vs the bot / vs older versions / vs itself (default 0.05 0.25 0.35 0.35)')
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
