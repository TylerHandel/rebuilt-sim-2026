#!/usr/bin/env python3
"""Train the end-to-end neural-network driver for the REBUILT simulator.

The network sees the field (tools: js/nn/obs.js) and drives the robot directly: movement,
rotation, intake, shoot, pass and outtake, 10 times a second, for the whole match.

How it uses the computer:
  * CPU: one Node.js worker per hardware thread (minus one) plays matches nonstop with the game's
    own physics. Each worker runs the network itself, so no worker ever waits for another.
  * GPU: PyTorch (CUDA) trains the network with PPO on the experience the workers send, while
    they keep playing. New weights go out to the workers between their chunks of experience.

Quick start (see tools/nn/README.md):
  python tools/nn/train.py                      # train (Ctrl+C saves and stops; --resume continues)
  python tools/nn/train.py --bc recordings      # first learn from your recorded driving
  python tools/nn/train.py --bc recordings --bc-only   # just clone your driving and export it
The current network is published to js/nn/driver.json; refresh the game to drive with it.
"""
import argparse
import base64
import collections
import json
import math
import os
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

ROOT = Path(__file__).resolve().parents[2]
REW_N = 5  # per step from the workers: dOwnPts, dOppPts, dIntaked, dInactiveFuel, dFoulPtsGiven
OBS_VERSION = 2
ACT_CONT, ACT_BIN = 3, 4

# Opponent curriculum: a level unlocks once the network wins >= 55% against everything in the
# level before it. Scripted AIs are the game's OpponentAI (strategy, skill).
LEVELS = [
    [('off', None, None)],
    [('ai', 'scorer', 'rookie')],
    [('ai', 'defense', 'rookie'), ('ai', 'hybrid', 'rookie')],
    [('ai', 'scorer', 'regional'), ('ai', 'defense', 'regional'), ('ai', 'hybrid', 'regional')],
    [('ai', 'scorer', 'champs'), ('ai', 'defense', 'champs'), ('ai', 'hybrid', 'champs')],
    [('ai', 'scorer', 'trained'), ('ai', 'defense', 'trained'), ('ai', 'hybrid', 'trained')],
]
SOLO_TARGET = 100  # level 0 counts as "won" when it scores this many points alone
SELFPLAY_LEVEL = 3  # snapshots of itself join the opponent pool from this level on


def okey(o):
    return o[0] if o[0] == 'off' else f'{o[1]}:{o[2]}'


# ---------------------------------------------------------------------------------- networks
def mlp(sizes, act=nn.Tanh, out_gain=1.0):
    layers = []
    for i in range(len(sizes) - 1):
        lin = nn.Linear(sizes[i], sizes[i + 1])
        last = i == len(sizes) - 2
        nn.init.orthogonal_(lin.weight, out_gain if last else math.sqrt(2))
        nn.init.zeros_(lin.bias)
        layers.append(lin)
        if not last:
            layers.append(act())
    return nn.Sequential(*layers)


class Actor(nn.Module):
    def __init__(self, obs_dim, hidden):
        super().__init__()
        self.hidden = list(hidden)
        self.body = nn.Sequential(*mlp([obs_dim, *hidden]), nn.Tanh())  # hidden layers, each tanh
        self.mu = nn.Linear(hidden[-1], ACT_CONT)
        self.lg = nn.Linear(hidden[-1], ACT_BIN)
        nn.init.orthogonal_(self.mu.weight, 0.01)
        nn.init.zeros_(self.mu.bias)
        nn.init.orthogonal_(self.lg.weight, 0.01)
        # start out holding the intake, not shooting, passing or spitting FUEL out
        with torch.no_grad():
            self.lg.bias.copy_(torch.tensor([1.0, -1.0, -2.5, -3.5]))
        self.log_std = nn.Parameter(torch.full((ACT_CONT,), -0.5))

    def forward(self, x):
        h = self.body(x)
        return self.mu(h), self.lg(h)

    def dist_terms(self, x, act):
        mu, lg = self(x)
        ls = self.log_std.clamp(-3.0, 0.5)
        a, b = act[:, :ACT_CONT], act[:, ACT_CONT:]
        lp_c = (-0.5 * ((a - mu) / ls.exp()) ** 2 - ls - 0.5 * math.log(2 * math.pi)).sum(-1)
        lp_b = -nn.functional.binary_cross_entropy_with_logits(lg, b, reduction='none').sum(-1)
        ent_c = (ls + 0.5 * math.log(2 * math.pi * math.e)).sum().expand(x.shape[0])
        p = torch.sigmoid(lg)
        ent_b = nn.functional.binary_cross_entropy_with_logits(lg, p, reduction='none').sum(-1)
        return lp_c + lp_b, ent_c + ent_b, mu, lg


class RunningNorm:
    """Running mean / variance of the observations (exported with the weights)."""

    def __init__(self, n):
        self.mean = np.zeros(n, np.float64)
        self.var = np.ones(n, np.float64)
        self.count = 1e-4

    def update(self, x):
        bm, bv, bc = x.mean(0), x.var(0), x.shape[0]
        d = bm - self.mean
        tot = self.count + bc
        self.mean = self.mean + d * bc / tot
        self.var = (self.var * self.count + bv * bc + d * d * self.count * bc / tot) / tot
        self.count = tot

    @property
    def std(self):
        return np.sqrt(self.var + 1e-8)

    def apply(self, x):
        return np.clip((x - self.mean) / self.std, -10, 10).astype(np.float32)

    def state(self):
        return {'mean': self.mean.tolist(), 'var': self.var.tolist(), 'count': self.count}

    def load(self, s):
        self.mean, self.var, self.count = np.array(s['mean']), np.array(s['var']), s['count']


def export_policy(actor, norm, path, version, info):
    """Write the actor in the JSON format js/nn/policy.js reads."""
    lins = [m for m in actor.body if isinstance(m, nn.Linear)]
    parts = []
    for m in [*lins, actor.mu, actor.lg]:
        parts += [m.weight.detach().float().cpu().numpy().ravel(), m.bias.detach().float().cpu().numpy().ravel()]
    blob = np.concatenate(parts).astype('<f4').tobytes()
    data = {
        'format': 'rebuilt-nn-policy', 'obsVersion': OBS_VERSION, 'obsDim': lins[0].in_features,
        'version': version, 'hidden': actor.hidden, 'activation': 'tanh',
        'obsMean': [float(v) for v in norm.mean], 'obsStd': [float(v) for v in norm.std],
        'logStd': [float(v) for v in actor.log_std.detach().clamp(-3.0, 0.5).cpu()],
        'weights': base64.b64encode(blob).decode('ascii'), 'info': info,
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data))
    os.replace(tmp, path)


# ---------------------------------------------------------------------------------- workers
class Workers:
    def __init__(self, n, chunk_dir, low_priority=True):
        node = shutil.which('node')
        if not node:
            sys.exit('Node.js was not found on PATH. Install it from https://nodejs.org (LTS) and run "npm install" in the repo.')
        if not (ROOT / 'node_modules' / '@dimforge').exists():
            sys.exit('Missing node_modules: run "npm install" in the repo folder first.')
        self.q = queue.Queue()
        self.procs = []
        env_hook = (ROOT / 'tools' / 'node-env.mjs').as_uri()
        # own process group: Ctrl+C in the console reaches only the trainer, which saves and then
        # stops the workers itself
        flags = 0
        if os.name == 'nt':
            flags = subprocess.CREATE_NEW_PROCESS_GROUP
            if low_priority:
                flags |= subprocess.BELOW_NORMAL_PRIORITY_CLASS  # keep the desktop responsive
        for i in range(n):
            p = subprocess.Popen(
                [node, '--import', env_hook, str(ROOT / 'tools' / 'nn' / 'worker.mjs'), '--id', str(i), '--dir', str(chunk_dir)],
                cwd=str(ROOT), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1, creationflags=flags, start_new_session=os.name != 'nt',
                preexec_fn=(lambda: os.nice(5)) if (os.name != 'nt' and low_priority) else None,
            )
            self.procs.append(p)
            threading.Thread(target=self._read, args=(i, p.stdout), daemon=True).start()
            threading.Thread(target=self._err, args=(i, p.stderr), daemon=True).start()

    def _read(self, i, f):
        for line in f:
            try:
                self.q.put(json.loads(line))
            except json.JSONDecodeError:
                pass
        self.q.put({'type': 'exit', 'id': i})

    def _err(self, i, f):
        for line in f:
            line = line.rstrip()
            if line and 'ExperimentalWarning' not in line:
                print(f'[worker {i}] {line}', file=sys.stderr)

    def broadcast(self, msg):
        line = json.dumps(msg) + '\n'
        for p in self.procs:
            if p.poll() is None:
                try:
                    p.stdin.write(line)
                    p.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass

    def close(self):
        self.broadcast({'cmd': 'stop'})
        t0 = time.time()
        for p in self.procs:
            try:
                p.wait(timeout=max(0.1, 10 - (time.time() - t0)))
            except subprocess.TimeoutExpired:
                p.kill()


def read_chunk(msg):
    n, D, A, R = msg['n'], msg['D'], msg['A'], msg['R']
    raw = np.fromfile(msg['file'], dtype='<f4')
    try:
        os.remove(msg['file'])
    except OSError:
        pass
    o = 0

    def take(k):
        nonlocal o
        a = raw[o:o + k]
        o += k
        return a

    return {
        'obs': take(n * D).reshape(n, D), 'act': take(n * A).reshape(n, A), 'logp': take(n).copy(),
        'rew': take(n * R).reshape(n, R), 'done': take(n).copy(), 'next': take(D).copy(),
        'version': msg['version'],
    }


def load_demos(folder, obs_dim):
    """Recordings exported from the game (Record for neural net)."""
    obs, act = [], []
    for f in sorted(Path(folder).glob('*.json')):
        d = json.loads(f.read_text())
        if d.get('format') != 'rebuilt-nn-demo':
            continue
        if d['obsVersion'] != OBS_VERSION or d['obsDim'] != obs_dim:
            print(f'skipping {f.name}: recorded with observation v{d["obsVersion"]}')
            continue
        o = np.frombuffer(base64.b64decode(d['obs']), '<f4').reshape(-1, obs_dim)
        a = np.frombuffer(base64.b64decode(d['act']), '<f4').reshape(-1, ACT_CONT + ACT_BIN)
        obs.append(o)
        act.append(a)
        print(f'  {f.name}: {len(o)} samples ({len(o) / 600:.1f} min of driving)')
    if not obs:
        return None
    return np.concatenate(obs), np.concatenate(act)


# ---------------------------------------------------------------------------------- trainer
class Trainer:
    def __init__(self, a):
        self.a = a
        self.dev = torch.device(a.device if a.device != 'auto' else ('cuda' if torch.cuda.is_available() else 'cpu'))
        if self.dev.type == 'cuda':
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
        self.run = ROOT / 'runs' / a.run
        self.run.mkdir(parents=True, exist_ok=True)
        self.obs_dim = None
        self.version = 0
        self.steps = 0
        self.updates = 0
        self.level = 0
        self.t_start = time.time()
        self.episodes = 0
        self.hist = collections.defaultdict(lambda: collections.deque(maxlen=100))  # opp key -> margins
        self.own_hist = collections.defaultdict(lambda: collections.deque(maxlen=100))
        self.pool = []  # self-play snapshot paths
        self.demos = None

    # ---- setup
    def build(self, obs_dim):
        a = self.a
        self.obs_dim = obs_dim
        self.actor = Actor(obs_dim, a.hidden).to(self.dev)
        self.critic = mlp([obs_dim, *a.critic, 1]).to(self.dev)
        self.opt = torch.optim.Adam([*self.actor.parameters(), *self.critic.parameters()], lr=a.lr, eps=1e-5)
        self.norm = RunningNorm(obs_dim)

    def save(self):
        ck = {
            'actor': self.actor.state_dict(), 'critic': self.critic.state_dict(), 'opt': self.opt.state_dict(),
            'norm': self.norm.state(), 'version': self.version, 'steps': self.steps, 'updates': self.updates,
            'level': self.level, 'pool': self.pool, 'obs_dim': self.obs_dim, 'hidden': self.a.hidden,
            'critic_sizes': self.a.critic, 'episodes': self.episodes,
            'hist': {k: list(v) for k, v in self.hist.items()}, 'own_hist': {k: list(v) for k, v in self.own_hist.items()},
            'elapsed': self.elapsed(),
        }
        tmp = self.run / 'ckpt.tmp'
        torch.save(ck, tmp)
        os.replace(tmp, self.run / 'ckpt.pt')

    def load(self):
        ck = torch.load(self.run / 'ckpt.pt', map_location=self.dev, weights_only=False)
        self.a.hidden, self.a.critic = ck['hidden'], ck['critic_sizes']
        self.build(ck['obs_dim'])
        self.actor.load_state_dict(ck['actor'])
        self.critic.load_state_dict(ck['critic'])
        self.opt.load_state_dict(ck['opt'])
        for g in self.opt.param_groups:
            g['lr'] = self.a.lr
        self.norm.load(ck['norm'])
        self.version, self.steps, self.updates = ck['version'], ck['steps'], ck['updates']
        self.level, self.pool, self.episodes = ck['level'], [p for p in ck['pool'] if Path(p).exists()], ck['episodes']
        for k, v in ck['hist'].items():
            self.hist[k].extend(v)
        for k, v in ck['own_hist'].items():
            self.own_hist[k].extend(v)
        self.t_start = time.time() - ck.get('elapsed', 0)
        print(f'Resumed {self.run.name}: {self.steps:,} steps, {self.updates} updates, level {self.level}')

    def elapsed(self):
        return time.time() - self.t_start

    def info(self):
        return {
            'run': self.a.run, 'steps': self.steps, 'updates': self.updates, 'level': self.level,
            'hours': round(self.elapsed() / 3600, 2), 'episodes': self.episodes,
            'date': time.strftime('%Y-%m-%d %H:%M'), 'device': str(self.dev),
            'vs': {k: round(float(np.mean(v)), 1) for k, v in self.hist.items() if v},
        }

    def publish(self):
        cur = self.run / 'policy' / 'current.json'
        export_policy(self.actor, self.norm, cur, self.version, self.info())
        if self.a.publish:
            shutil.copyfile(cur, ROOT / 'js' / 'nn' / 'driver.json')
        return cur

    # ---- opponents
    def opponents(self):
        lv = min(self.level, len(LEVELS) - 1)
        out = []
        for li in range(lv + 1):
            for o in LEVELS[li]:
                k = okey(o)
                h = self.hist[k]
                wr = (sum(m > 0 for m in h) / len(h)) if len(h) >= 10 else 0.5
                if o[0] == 'off':
                    w = 1.0 if lv == 0 else 0.1
                else:
                    # prioritize the opponents it doesn't beat yet; older levels still get some play
                    w = (1.0 - wr + 0.15) ** 2 * (1.0 if li == lv else 0.5)
                out.append({'key': k, 'kind': o[0], 'strategy': o[1], 'skill': o[2], 'weight': w})
        if self.level >= SELFPLAY_LEVEL and self.pool:
            tot = sum(o['weight'] for o in out)
            share = tot * 0.4 / len(self.pool)  # self-play is ~30% of matches
            for p in self.pool:
                out.append({'key': 'self', 'kind': 'nn', 'path': str(p), 'weight': share})
        return out

    def check_level(self):
        if self.level >= len(LEVELS) - 1:
            return
        ok = True
        for o in LEVELS[self.level]:
            k = okey(o)
            if o[0] == 'off':
                h = self.own_hist[k]
                ok &= len(h) >= 40 and float(np.mean(h)) >= SOLO_TARGET
            else:
                h = self.hist[k]
                ok &= len(h) >= 40 and sum(m > 0 for m in h) / len(h) >= 0.55
        if ok:
            self.level += 1
            names = ', '.join(okey(o) for o in LEVELS[self.level])
            print(f'\n*** Level {self.level} unlocked: now also playing {names}\n')

    def snapshot(self):
        d = self.run / 'pool'
        d.mkdir(exist_ok=True)
        p = d / f'v{self.version}.json'
        export_policy(self.actor, self.norm, p, self.version, self.info())
        self.pool.append(str(p))
        while len(self.pool) > self.a.pool_size:
            old = self.pool.pop(0)
            try:
                os.remove(old)
            except OSError:
                pass

    # ---- behavior cloning (learning from recorded human driving)
    def bc(self, obs, act, epochs):
        a = self.a
        self.norm.update(obs)
        X = torch.tensor(self.norm.apply(obs), device=self.dev)
        Y = torch.tensor(act, device=self.dev)
        opt = torch.optim.Adam(self.actor.parameters(), lr=1e-3)
        n = len(X)
        print(f'Learning from {n:,} recorded decisions ({n / 600:.1f} min) on {self.dev} ...')
        for ep in range(epochs):
            perm = torch.randperm(n, device=self.dev)
            tot = 0.0
            for i in range(0, n, 1024):
                idx = perm[i:i + 1024]
                mu, lg = self.actor(X[idx])
                loss = ((mu - Y[idx, :ACT_CONT]) ** 2).sum(-1).mean() + nn.functional.binary_cross_entropy_with_logits(lg, Y[idx, ACT_CONT:], reduction='none').sum(-1).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                tot += loss.item() * len(idx)
            if ep % max(1, epochs // 10) == 0 or ep == epochs - 1:
                print(f'  epoch {ep + 1}/{epochs}  loss {tot / n:.4f}')
        # a cloned policy acts with little noise at first so self-play starts out driving like you
        with torch.no_grad():
            self.actor.log_std.fill_(-1.2)
        self.demos = (X, Y)

    def bc_loss(self, mb):
        X, Y = self.demos
        idx = torch.randint(0, len(X), (mb,), device=self.dev)
        mu, lg = self.actor(X[idx])
        return ((mu - Y[idx, :ACT_CONT]) ** 2).sum(-1).mean() + nn.functional.binary_cross_entropy_with_logits(lg, Y[idx, ACT_CONT:], reduction='none').sum(-1).mean()

    # ---- PPO
    def reward(self, rew):
        a = self.a
        # main objective: win the match (own points - their points, fouls included)
        r = (rew[:, 0] - rew[:, 1]) / 10.0
        # early shaping that fades out: pick up FUEL, don't shoot into an inactive HUB
        k = max(0.0, 1.0 - self.steps / a.shaping_steps) if a.shaping_steps > 0 else 0.0
        r = r + k * (0.03 * rew[:, 2] - 0.02 * rew[:, 3])
        return r.astype(np.float32)

    def update(self, chunks):
        a = self.a
        obs = np.concatenate([c['obs'] for c in chunks])
        self.norm.update(obs)
        with torch.no_grad():
            V = lambda x: self.critic(torch.tensor(self.norm.apply(x), device=self.dev)).squeeze(-1).cpu().numpy()
            adv_l, ret_l = [], []
            for c in chunks:
                v = V(c['obs'])
                vn = V(c['next'][None])[0]
                r, d = self.reward(c['rew']), c['done']
                n = len(r)
                adv = np.zeros(n, np.float32)
                last = 0.0
                for t in range(n - 1, -1, -1):
                    nv = vn if t == n - 1 else v[t + 1]
                    nonterm = 1.0 - d[t]
                    delta = r[t] + a.gamma * nv * nonterm - v[t]
                    last = delta + a.gamma * a.lam * nonterm * last
                    adv[t] = last
                adv_l.append(adv)
                ret_l.append(adv + v)
        X = torch.tensor(self.norm.apply(obs), device=self.dev)
        ACT = torch.tensor(np.concatenate([c['act'] for c in chunks]), device=self.dev)
        LP = torch.tensor(np.concatenate([c['logp'] for c in chunks]), device=self.dev)
        ADV = torch.tensor(np.concatenate(adv_l), device=self.dev)
        RET = torch.tensor(np.concatenate(ret_l), device=self.dev)
        n = len(X)
        stats = collections.defaultdict(list)
        bc_w = a.bc_weight * max(0.0, 1.0 - self.steps / a.bc_steps) if self.demos is not None and a.bc_steps > 0 else 0.0
        for _ in range(a.epochs):
            perm = torch.randperm(n, device=self.dev)
            for i in range(0, n, a.minibatch):
                idx = perm[i:i + a.minibatch]
                lp, ent, _, _ = self.actor.dist_terms(X[idx], ACT[idx])
                ratio = (lp - LP[idx]).exp()
                ad = ADV[idx]
                ad = (ad - ad.mean()) / (ad.std() + 1e-8)
                pl = -torch.min(ratio * ad, ratio.clamp(1 - a.clip, 1 + a.clip) * ad).mean()
                vl = 0.5 * ((self.critic(X[idx]).squeeze(-1) - RET[idx]) ** 2).mean()
                loss = pl + a.vf * vl - a.ent * ent.mean()
                if bc_w > 0:
                    bl = self.bc_loss(len(idx))
                    loss = loss + bc_w * bl
                    stats['bc'].append(bl.item())
                self.opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_([*self.actor.parameters(), *self.critic.parameters()], 0.5)
                self.opt.step()
                with torch.no_grad():
                    stats['pi'].append(pl.item())
                    stats['v'].append(vl.item())
                    stats['ent'].append(ent.mean().item())
                    stats['kl'].append((LP[idx] - lp).mean().item())
                    stats['clip'].append(((ratio - 1).abs() > a.clip).float().mean().item())
        self.version += 1
        self.updates += 1
        return {k: float(np.mean(v)) for k, v in stats.items()}

    # ---- main loop
    def train(self):
        a = self.a
        chunk_dir = self.run / 'chunks'
        shutil.rmtree(chunk_dir, ignore_errors=True)
        chunk_dir.mkdir(parents=True)
        nw = a.workers or max(1, (os.cpu_count() or 2) - 1)
        print(f'Device: {self.dev}' + (f' ({torch.cuda.get_device_name(0)})' if self.dev.type == 'cuda' else ''))
        print(f'Starting {nw} simulation workers (CPU threads: {os.cpu_count()}) ...')
        W = Workers(nw, chunk_dir, low_priority=not a.full_priority)
        ready = 0
        while ready < nw:
            m = W.q.get(timeout=120)
            if m['type'] == 'ready':
                ready += 1
                if self.obs_dim is None:
                    self.build(m['obsDim'])
            elif m['type'] == 'exit':
                sys.exit('a worker exited during startup (see the messages above)')
        if a.resume:
            self.load()
        elif a.bc:
            demos = load_demos(a.bc, self.obs_dim)
            if demos is None:
                sys.exit(f'No recordings found in {a.bc}')
            self.bc(*demos, a.bc_epochs)
        if self.a.resume and a.bc:
            demos = load_demos(a.bc, self.obs_dim)
            if demos is not None:
                self.demos = (torch.tensor(self.norm.apply(demos[0]), device=self.dev), torch.tensor(demos[1], device=self.dev))
        if a.level is not None:
            self.level = max(0, min(len(LEVELS) - 1, a.level))
        policy = self.publish()

        def config():
            W.broadcast({'cmd': 'config', 'policy': str(policy), 'version': self.version, 'opponents': self.opponents(), 'robots': a.robots, 'chunk': a.chunk})

        config()
        stop = {'flag': False}

        def on_sigint(*_):
            if stop['flag']:
                raise KeyboardInterrupt
            stop['flag'] = True
            print('\nStopping after this update (Ctrl+C again to quit right away) ...')

        signal.signal(signal.SIGINT, on_sigint)
        log = open(self.run / 'progress.jsonl', 'a')
        pending, n_pending, n_errors = [], 0, 0
        recent_eps = collections.deque(maxlen=200)
        t_last, steps_last = time.time(), self.steps
        last_save = time.time()
        try:
            while not stop['flag'] and self.steps < a.total_steps:
                try:
                    m = W.q.get(timeout=1.0)
                except queue.Empty:
                    continue
                t = m['type']
                if t == 'chunk':
                    c = read_chunk(m)
                    if c['version'] >= self.version - a.max_lag:
                        pending.append(c)
                        n_pending += len(c['obs'])
                elif t == 'episode':
                    self.episodes += 1
                    self.hist[m['opp']].append(m['margin'])
                    self.own_hist[m['opp']].append(m['own'])
                    recent_eps.append(m)
                elif t == 'error':
                    print(f'[worker {m["id"]}] error: {m["msg"]}', file=sys.stderr)
                    n_errors += 1
                    if n_errors >= 3 * nw and self.episodes == 0:
                        print('Every match is failing (see the errors above); stopping.', file=sys.stderr)
                        break
                elif t == 'exit':
                    print(f'worker {m["id"]} exited', file=sys.stderr)
                if n_pending < a.batch:
                    continue
                t0 = time.time()
                st = self.update(pending)
                self.steps += n_pending
                upd_s = time.time() - t0
                pending, n_pending = [], 0
                self.check_level()
                if self.level >= SELFPLAY_LEVEL and (not self.pool or self.updates % a.snapshot_every == 0):
                    self.snapshot()
                policy = self.publish()
                config()
                now = time.time()
                sps = (self.steps - steps_last) / max(1e-6, now - t_last)
                t_last, steps_last = now, self.steps
                eps = list(recent_eps)
                by = collections.defaultdict(list)
                for e in eps:
                    by[e['opp']].append(e)
                vs = '  '.join(f'{k} {np.mean([e["margin"] for e in v]):+.0f} ({sum(e["margin"] > 0 for e in v)}/{len(v)})' for k, v in sorted(by.items()))
                own = np.mean([e['own'] for e in eps]) if eps else 0
                gpu = f'  gpu {torch.cuda.max_memory_allocated() / 2**20:.0f}MB' if self.dev.type == 'cuda' else ''
                print(f'[{self.elapsed() / 3600:5.2f}h] upd {self.updates:4d}  steps {self.steps / 1e6:7.2f}M  {sps:5.0f} steps/s ({sps * 0.1 / 160 * 3600:.0f} matches/h)  '
                      f'lvl {self.level}  pts {own:5.1f}  ent {st["ent"]:+.2f}  kl {st["kl"]:+.4f}  upd {upd_s:.1f}s{gpu}')
                if vs:
                    print(f'      vs: {vs}')
                rec = {'t': round(self.elapsed(), 1), 'steps': self.steps, 'updates': self.updates, 'level': self.level, 'sps': round(sps, 1), 'own': round(float(own), 2), **{k: round(v, 5) for k, v in st.items()},
                       'vs': {k: round(float(np.mean([e['margin'] for e in v])), 2) for k, v in by.items()},
                       'win': {k: round(sum(e['margin'] > 0 for e in v) / len(v), 3) for k, v in by.items()}}
                log.write(json.dumps(rec) + '\n')
                log.flush()
                recent_eps.clear()
                if time.time() - last_save > a.save_minutes * 60:
                    self.save()
                    last_save = time.time()
        finally:
            print('Saving ...')
            self.save()
            self.publish()
            W.close()
            log.close()
            shutil.rmtree(chunk_dir, ignore_errors=True)
            print(f'Saved {self.run / "ckpt.pt"}' + (' and js/nn/driver.json' if a.publish else ''))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--run', default='driver', help='run name (folder under runs/)')
    p.add_argument('--resume', action='store_true', help='continue the run from runs/<run>/ckpt.pt')
    p.add_argument('--workers', type=int, default=0, help='simulation workers (default: CPU threads - 1)')
    p.add_argument('--device', default='auto', help='cuda, cpu or auto')
    p.add_argument('--full-priority', action='store_true', help="don't lower the workers' CPU priority")
    p.add_argument('--total-steps', type=float, default=float('inf'))
    p.add_argument('--level', type=int, default=None, help='start (or continue) at this opponent level (0-5)')
    p.add_argument('--batch', type=int, default=16384, help='decisions per PPO update')
    p.add_argument('--chunk', type=int, default=256, help='decisions per worker chunk')
    p.add_argument('--minibatch', type=int, default=2048)
    p.add_argument('--epochs', type=int, default=4)
    p.add_argument('--lr', type=float, default=3e-4)
    p.add_argument('--gamma', type=float, default=0.995)
    p.add_argument('--lam', type=float, default=0.95)
    p.add_argument('--clip', type=float, default=0.2)
    p.add_argument('--ent', type=float, default=0.003)
    p.add_argument('--vf', type=float, default=0.5)
    p.add_argument('--max-lag', type=int, default=1, help='oldest policy version (behind the current one) whose experience is used')
    p.add_argument('--hidden', type=int, nargs='+', default=[512, 256], help='actor hidden layers (runs in the browser)')
    p.add_argument('--critic', type=int, nargs='+', default=[1024, 512, 256], help='critic hidden layers (GPU only)')
    p.add_argument('--shaping-steps', type=float, default=20e6, help='steps over which the pickup bonus fades out')
    p.add_argument('--robots', nargs='+', default=['2910', '4414', '8793'], help='robots it learns to drive')
    p.add_argument('--snapshot-every', type=int, default=25, help='updates between self-play snapshots')
    p.add_argument('--pool-size', type=int, default=8, help='self-play snapshots kept')
    p.add_argument('--save-minutes', type=float, default=10)
    p.add_argument('--no-publish', dest='publish', action='store_false', help="don't write js/nn/driver.json")
    p.add_argument('--bc', help='folder of recordings (exported from the game) to learn from first')
    p.add_argument('--bc-epochs', type=int, default=30)
    p.add_argument('--bc-only', action='store_true', help='only clone the recordings, export and exit')
    p.add_argument('--bc-weight', type=float, default=0.5, help='keep imitating the recordings this much during PPO ...')
    p.add_argument('--bc-steps', type=float, default=10e6, help='... fading out over this many steps')
    a = p.parse_args()

    tr = Trainer(a)
    if a.bc_only:
        if not a.bc:
            sys.exit('--bc-only needs --bc <folder>')
        from_obs = OBS_DIM_FROM_JS()
        tr.build(from_obs)
        demos = load_demos(a.bc, from_obs)
        if demos is None:
            sys.exit(f'No recordings found in {a.bc}')
        tr.bc(*demos, a.bc_epochs)
        tr.publish()
        print('Exported the cloned driver' + (' to js/nn/driver.json' if a.publish else ''))
        return
    tr.train()


def OBS_DIM_FROM_JS():
    """Ask Node for the observation size (it is defined in js/nn/obs.js)."""
    node = shutil.which('node')
    out = subprocess.run([node, '--import', (ROOT / 'tools' / 'node-env.mjs').as_uri(), '-e',
                          "import('" + (ROOT / 'js' / 'nn' / 'obs.js').as_uri() + "').then(m => console.log(m.OBS_DIM))"],
                         cwd=str(ROOT), capture_output=True, text=True, check=True)
    return int(out.stdout.strip().splitlines()[-1])


if __name__ == '__main__':
    main()
