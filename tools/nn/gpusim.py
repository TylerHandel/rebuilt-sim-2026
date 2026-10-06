"""Batched REBUILT simulator on the GPU (PyTorch), for training the neural-net driver fast.

Thousands of matches run at once as tensor math, 1v1 up to 3v3. It is a simplified version of
the real game:
  * robots: swerve drive with the real speed / acceleration / turn limits; box-shaped bodies that
    collide with the walls, HUBS, TRENCH posts, TOWERS (and TRENCH arms for tall robots) and each
    other (pushing, blocking); intake, hopper capacity (packing hoppers jam at ~80% like the real
    ones), flywheel spin-up, turret / chassis aiming, shot rate
  * FUEL: rolls and slows on the carpet, bounces off walls and field elements, gets pushed (and
    pushes back on the robot); crowded FUEL spreads out instead of stacking up; shots, passes,
    misses and the HUB exits
  * match: AUTO, the 3 s gap, TELEOP shifts with the active-HUB rules, 3 s scoring grace, human
    players throwing from the CHUTE
  * domain randomization: each robot's speed, acceleration, intake and accuracy and the carpet's
    rolling resistance vary a little per match, so what it learns doesn't hinge on exact numbers
What the network sees is built exactly like js/nn/obs.js, and it gives the same 7 controls, so a
network trained here drives the real game (and is fine-tuned there with tools/nn/train.py).

Robot slots: 0-2 are alliance A, 3-5 alliance B. A match has 1-3 robots per alliance. Who
alliance B is (nobody, scripted bots, older networks, the network itself) is fixed per match
slot: set_mix() splits the matches between them.

Every step works on arrays of the same size every time and never waits for the CPU (no
data-dependent shapes), and the state lives in fixed buffers: so a whole decision can be
recorded once as a CUDA graph and replayed, and torch.compile can fuse it (see compile()).

Robot specs, the field and the FUEL layout come from tools/nn/gpusim-params.json, written by
tools/nn/gpusim-export.mjs from the real game.
"""
import json
import math
from pathlib import Path

import numpy as np
import torch

PARAMS = Path(__file__).with_name('gpusim-params.json')

FIELD, HELD, FLY, HUBQ, CHUTE, DEAD = 0, 1, 2, 3, 4, 5   # DEAD: the spare FUEL slot that masked writes go to
FLY_HIT, FLY_MISS, FLY_LAND = 0, 1, 2
OPP_NONE, OPP_BOT, OPP_SNAP, OPP_SELF = 0, 1, 2, 3
SUB = 4      # physics substeps per decision
RS = 6       # robot slots per match
TEAM = torch.tensor([0, 0, 0, 1, 1, 1])
PACK_FULL = 0.8  # packing hoppers jam at about this fraction of their listed capacity
INTAKE_MAX = 70.0  # FUEL/s a real intake swallows driving through a pile (tools/intake-test.mjs)
CONTACTS = 64  # FUEL a robot can touch at once (more than fit around a bumper)
LIVE_HEAD, LIVE_ROBOT = 6, 8


def wrap(a):
    return torch.remainder(a + math.pi, 2 * math.pi) - math.pi


def approach(cur, tgt, step):
    return cur + torch.clamp(tgt - cur, -step, step)


class GpuSim:
    def __init__(self, n, device='cuda', robots=None, seed=0, params=PARAMS, substeps=SUB, team_probs=(0.25, 0.15, 0.6), randomize=True):
        P = json.loads(Path(params).read_text())
        self.P = P
        self.n = n
        self.sub = substeps
        self.randomize = randomize
        self.dev = torch.device(device)
        torch.manual_seed(seed)  # the default generator: CUDA graphs can record it
        d = self.dev
        f = P['field']
        self.HL, self.HW = f['halfL'], f['halfW']
        self.lineX = f['allianceLineX']
        self.hubX = f['hubX']
        self.hs = P['hub']['size'] / 2
        self.R = P['fuel']['radius']
        self.dt = P['obs']['decisionDt']
        self.D = P['obs']['dim']
        self.layout = P['obs']['layout']
        T = P['timing']
        self.T_AUTO, self.T_GAP, self.T_TELE, self.T_POST = T['auto'], T['autoGap'], T['teleop'], T['post']
        self.T_TRANS, self.T_SHIFT, self.T_END = T['transition'], T['shift'], T['endgame']
        self.TOTAL = self.T_AUTO + self.T_TELE
        self.T_DONE = self.T_AUTO + self.T_GAP + self.T_TELE + self.T_POST
        self.grace = P['hub']['scoreGrace']
        self.team = TEAM.to(d)
        self.team_probs = torch.tensor(team_probs, device=d, dtype=torch.float)
        self.pfsp_p = torch.ones(1, device=d)  # which older network a match plays (see set_snapshots)

        # ---- robot specs, indexed by robot type
        self.order = P['robotOrder']
        self.NR = len(self.order)
        spec = lambda k: torch.tensor([float(P['robots'][r][k]) for r in self.order], device=d)
        self.S = {k: spec(k) for k in ['halfL', 'halfW', 'maxSpeed', 'maxAccel', 'maxOmega', 'maxAlpha', 'intakeWidth', 'intakeReach',
                                       'intakeRate', 'deployTime', 'capIn', 'capOut', 'capMax', 'bps', 'spinTau', 'speedMax',
                                       'turretRate', 'turretRange', 'aimOffset', 'turrets', 'mass']}
        self.S['turret'] = spec('turret') > 0.5
        self.S['latched'] = spec('latched') > 0.5
        self.S['fits'] = spec('fitsTrench') > 0.5
        self.S['packs'] = spec('packs') > 0.5
        tab = lambda k: torch.tensor([[float('nan') if v is None else v for v in P['robots'][r][k]] for r in self.order], device=d)
        self.hubV, self.passV = tab('hubV'), tab('passV')
        # share of shots that score, by robot, at 0 / 33 / 66 / 100% of its top speed (measured in
        # the full game by tools/nn/shoot-test.mjs: chassis-aimed shooters miss a lot on the move)
        cal_p = Path(params).with_name('shoot-calib.json')
        cal = json.loads(cal_p.read_text()) if cal_p.exists() else {}
        fr = ['0', '0.33', '0.66', '1']
        self.shotAcc = torch.tensor([[(cal.get(r, {}).get(f, {}).get('acc') or 0.95) for f in fr] for r in self.order], device=d)
        self.starts = torch.tensor([[P['robots'][r]['starts'][k] for k in P['startOrder']] for r in self.order], device=d)  # [NR,5,2] blue
        self.allowed = torch.tensor([self.order.index(r) for r in (robots or self.order)], device=d)

        # ---- field geometry
        self.obs_rects = torch.tensor([[o['x'], o['z'], o['hx'], o['hz']] for o in P['obstacles']], device=d)
        self.arms = torch.tensor([[a['x'], a['z'], a['hx'], a['hz']] for a in P['trenchArms']], device=d)
        self._rect_grid()
        self.chuteN = P['fuel']['perChute']
        self.pre = P['fuel']['preload']
        self.B = P['fuel']['total']
        self.BB = self.B + 1                      # + the spare slot (index B) masked writes go to
        self._fuel_templates(P['fuel']['layouts'])
        thr = P['outposts']
        self.hpThrow = torch.tensor([[thr['blue']['throw'][0], thr['blue']['throw'][2]], [thr['red']['throw'][0], thr['red']['throw'][2]]], device=d)
        self.exitW = P['hub']['exitWidth']
        self.exitSpeed = (1.0, 2.2)

        # constants used inside the step (made once: creating a tensor from Python data mid-step
        # would be a copy from the CPU, which a CUDA graph can't record)
        self._ar_rs = torch.arange(RS, device=d)
        self._ar_bb = torch.arange(self.BB, device=d)
        self._ar_bb16 = self._ar_bb.to(torch.int16)
        # which robot (one-hot, obs.js): the robots observation v3 knew keep their slots there; ones
        # added since (v4) show as none of them and get a slot in the block at the end
        ob = P['obs']
        self.NOH = len(ob.get('v3Robots', self.order))
        self.NEW = ob.get('extraRobots', 0)
        assert self.order[:self.NOH] == ob.get('v3Robots', self.order), 'robotOrder must start with the v3 robots'
        self._ar_nr = torch.arange(self.NOH, device=d)
        self._ar_new = torch.arange(self.NOH, self.NOH + self.NEW, device=d)
        self._ar_k = torch.arange(CONTACTS, device=d)
        self._spare = self._ar_bb == self.B
        self._park = torch.stack([torch.zeros(RS, device=d), 50.0 + self._ar_rs.float()], -1)
        self._hubw = torch.tensor([[self.hubX, 0.0], [-self.hubX, 0.0]], device=d)
        self._ball_lim = torch.tensor([self.HL - self.R, self.HW - self.R], device=d)
        self._eye = torch.eye(RS, dtype=torch.bool, device=d)

        self._sub = self._substep
        self._reset_fn = self._reset
        self._alloc()
        self.set_mix([0.3, 0.7, 0.0, 0.0])
        self.reset(torch.ones(n, dtype=torch.bool, device=d))

    # ------------------------------------------------------------------ helpers
    def rand(self, *shape):
        return torch.rand(*shape, device=self.dev)

    def randn(self, *shape):
        return torch.randn(*shape, device=self.dev)

    def spec(self, k):
        return self.S[k][self.rtype]  # [N,RS]

    def _rect_grid(self):
        """Which field element is nearest, on a 5 cm grid: a FUEL only checks that one."""
        c = 0.05
        gx = torch.arange(int(2 * self.HL / c) + 2, device=self.dev) * c - self.HL
        gz = torch.arange(int(2 * self.HW / c) + 2, device=self.dev) * c - self.HW
        p = torch.stack(torch.meshgrid(gx, gz, indexing='ij'), -1).view(-1, 1, 2)
        q = (p - self.obs_rects[:, :2]).abs() - self.obs_rects[:, 2:]
        sd = q.clamp(min=0).norm(dim=-1) + q.max(-1).values.clamp(max=0)
        self.grid_c, self.grid_nz = c, len(gz)
        self.grid_nx = len(gx)
        self.grid_rect = sd.argmin(-1)

    def _nearest_rect(self, p):
        ix = ((p[..., 0] + self.HL) / self.grid_c).round().long().clamp(0, self.grid_nx - 1)
        iz = ((p[..., 1] + self.HW) / self.grid_c).round().long().clamp(0, self.grid_nz - 1)
        return self.obs_rects[self.grid_rect[ix * self.grid_nz + iz]]

    def _fuel_templates(self, layouts):
        """The FUEL at the start of a match, per team size (the real game stages fewer FUEL in the
        NEUTRAL ZONE the more robots preload): positions, states, owners, and which robot slot
        preloads each FUEL (-1: none)."""
        BB, C, Pn = self.BB, self.chuteN, self.pre
        pos, st, own, pre = [], [], [], []
        for kk in (1, 2, 3):
            L = torch.tensor(layouts[str(kk)], device=self.dev)
            F = len(L)
            p = torch.zeros(BB, 2, device=self.dev)
            p[:F] = L
            p[self.B] = torch.tensor([0.0, 60.0])
            s = torch.full((BB,), FIELD, dtype=torch.long, device=self.dev)
            o = torch.zeros(BB, dtype=torch.long, device=self.dev)
            r_ = torch.full((BB,), -1, dtype=torch.long, device=self.dev)
            s[F:F + 2 * C] = CHUTE
            o[F + C:F + 2 * C] = 1
            j = F + 2 * C
            for r in [*range(kk), *range(3, 3 + kk)]:
                r_[j:j + Pn] = r
                j += Pn
            assert j == self.B, f'FUEL layout for {kk}v{kk} does not add up to {self.B}'
            s[self.B] = DEAD
            pos.append(p), st.append(s), own.append(o), pre.append(r_)
        self.tmpl_pos, self.tmpl_st, self.tmpl_own, self.tmpl_pre = (torch.stack(v) for v in (pos, st, own, pre))

    def _alloc(self):
        n, d, BB = self.n, self.dev, self.BB
        before = set(vars(self))
        z = lambda *s: torch.zeros(*s, device=d)
        zb = lambda *s: torch.zeros(*s, dtype=torch.bool, device=d)
        zl = lambda *s: torch.zeros(*s, dtype=torch.long, device=d)
        self.rtype = zl(n, RS)
        self.sgn = z(n, RS)
        self.present = zb(n, RS)
        self.k = zl(n) + 1                     # robots per alliance
        self.pos, self.vel = z(n, RS, 2), z(n, RS, 2)
        self.yaw, self.omega = z(n, RS), z(n, RS)
        self.deploy, self.hopper, self.fly, self.turret = z(n, RS), z(n, RS), z(n, RS), z(n, RS)
        self.stored, self.tokens, self.feedT = z(n, RS), z(n, RS), z(n, RS)
        self.ready, self.shot, self.feeding = zb(n, RS), zb(n, RS), zb(n, RS)
        self.holdV, self.holdT = z(n, RS), z(n, RS) - 99
        self.cmd = z(n, RS, 7)
        self.prev_a = z(n, RS, 3)          # last decision's drive / turn command (smoothness)
        self.intaked, self.shots, self.passes, self._prevIntaked = z(n, RS), z(n, RS), z(n, RS), z(n, RS)
        self.t = z(n)
        self.firstInactive = zl(n)
        self.prevActive = zb(n, 2)
        self.lastDeact = z(n, 2) - 1e9
        self.score = z(n, 2, 3)            # per alliance (0 blue, 1 red): autoFuel, teleFuel, inactiveFuel
        self.hpCool = z(n, 2)
        self.opp_kind = zl(n)
        self.snap_slot = zl(n)             # which older network (OPP_SNAP matches)
        self.bot_skill = z(n, RS) + 1
        self.bot_def = zb(n, RS)
        # domain randomization (multipliers / offsets per robot and per match)
        self.dr_speed, self.dr_accel, self.dr_intake = z(n, RS) + 1, z(n, RS) + 1, z(n, RS) + 1
        self.dr_hit, self.dr_roll = z(n, RS), z(n) + 1
        self.bpos, self.bvel = z(n, BB, 2), z(n, BB, 2)
        self.bst = zl(n, BB) + DEAD
        self.bown = zl(n, BB)
        self.btim = z(n, BB)
        self.bland = z(n, BB, 2)
        self.bkind = zl(n, BB)
        # the state's fixed buffers: steps compute new tensors, then copy them back into these
        self._bufs = {k: v for k, v in vars(self).items() if k not in before and torch.is_tensor(v)}

    def _commit(self):
        """Copy the new state into the fixed buffers (a CUDA graph reads and writes those)."""
        for k, buf in self._bufs.items():
            cur = getattr(self, k)
            if cur is not buf:
                buf.copy_(cur)
                setattr(self, k, buf)

    def _rebind(self):
        for k, buf in self._bufs.items():
            setattr(self, k, buf)

    def alliance_idx(self):
        return (self.sgn < 0).long()  # [N,RS] 0 blue 1 red

    def learning(self):
        """[N,RS]: robots driven by the policy being trained (alliance A, and B in self-play)."""
        return self.present & ((self.team == 0) | (self.opp_kind == OPP_SELF)[:, None])

    def set_mix(self, probs):
        """Split the matches between the kinds of alliance B (nobody, scripted bots, older networks,
        the network itself) in these proportions: contiguous ranges of matches, so each kind's
        robots are a fixed slice. Returns [(start, end)] per kind. Reset the matches after this."""
        p = np.asarray(probs, dtype=np.float64)
        p = p / p.sum()
        raw = p * self.n
        cnt = np.floor(raw).astype(int)
        for i in np.argsort(-(raw - cnt))[:self.n - cnt.sum()]:
            cnt[i] += 1
        b = np.concatenate([[0], np.cumsum(cnt)])
        kind = torch.zeros(self.n, dtype=torch.long)
        for k in range(4):
            kind[b[k]:b[k + 1]] = k
        self.opp_kind.copy_(kind.to(self.dev))
        self.kind_range = [(int(b[k]), int(b[k + 1])) for k in range(4)]
        return self.kind_range

    def set_snapshots(self, probs):
        """How often an OPP_SNAP match picks each older network (from its next reset on)."""
        p = torch.as_tensor(probs, dtype=torch.float, device=self.dev)
        if p.shape != self.pfsp_p.shape:
            self.pfsp_p = p.clone()
        else:
            self.pfsp_p.copy_(p)

    # ------------------------------------------------------------------ reset
    def reset(self, m):
        """Start a new match where m (bool [N]) is set. Every match computes a fresh start and only
        the set ones take it: no waiting for the CPU, so it can run inside a CUDA graph."""
        self._reset_fn(m)
        self._commit()

    def _reset(self, m):
        N, d, B = self.n, self.dev, self.B

        def sel(new, old):
            return torch.where(m.view(-1, *([1] * (old.dim() - 1))), new, old)

        cdf = self.team_probs.cumsum(0)
        k = torch.searchsorted(cdf / cdf[-1], self.rand(N), right=True).clamp(max=2) + 1
        kind = self.opp_kind
        within = (self._ar_rs % 3)[None] < k[:, None]
        pres = within & ((self.team == 0)[None] | (kind != OPP_NONE)[:, None])
        # robots: alliance A drives the allowed robots, alliance B any robot
        a_t = self.allowed[torch.randint(0, len(self.allowed), (N, RS), device=d)]
        o_t = torch.randint(0, self.NR, (N, RS), device=d)
        rtype = torch.where((self.team == 0)[None], a_t, o_t)
        sA = torch.where(self.rand(N) < 0.5, 1.0, -1.0)
        s = torch.where((self.team == 0)[None], sA[:, None], -sA[:, None])
        # distinct starting spots per alliance
        perm = torch.argsort(self.rand(N, 2, self.starts.shape[1]), -1)[..., :3]
        st = torch.cat([perm[:, 0], perm[:, 1]], 1)
        p = self.starts[rtype, st] * s[..., None]
        p = torch.where(pres[..., None], p, self._park)
        self.k, self.present, self.rtype, self.sgn = sel(k, self.k), sel(pres, self.present), sel(rtype, self.rtype), sel(s, self.sgn)
        self.pos = sel(p, self.pos)
        self.yaw = sel(torch.where(s > 0, 0.0, math.pi), self.yaw)
        for name in ['vel', 'omega', 'deploy', 'hopper', 'fly', 'turret', 'tokens', 'feedT', 'holdV', 'intaked', 'shots', 'passes',
                     '_prevIntaked', 'hpCool', 'cmd', 'prev_a', 't', 'score', 'btim', 'bvel', 'bkind']:
            cur = getattr(self, name)
            setattr(self, name, sel(torch.zeros_like(cur), cur))
        for name in ['ready', 'shot', 'feeding', 'prevActive']:
            cur = getattr(self, name)
            setattr(self, name, sel(torch.zeros_like(cur), cur))
        self.holdT = sel(torch.full_like(self.holdT, -99.0), self.holdT)
        self.lastDeact = sel(torch.full_like(self.lastDeact, -1e9), self.lastDeact)
        self.firstInactive = sel(torch.zeros_like(self.firstInactive), self.firstInactive)
        self.bot_skill = sel(0.75 + 0.25 * self.rand(N, RS), self.bot_skill)
        self.bot_def = sel(self.rand(N, RS) < 0.2, self.bot_def)
        pc = self.pfsp_p.cumsum(0)
        self.snap_slot = sel(torch.searchsorted(pc / pc[-1].clamp(min=1e-9), self.rand(N), right=True).clamp(max=len(pc) - 1), self.snap_slot)
        if self.randomize:
            self.dr_speed = sel(0.92 + 0.13 * self.rand(N, RS), self.dr_speed)
            self.dr_accel = sel(0.85 + 0.25 * self.rand(N, RS), self.dr_accel)
            self.dr_intake = sel(0.7 + 0.4 * self.rand(N, RS), self.dr_intake)
            self.dr_hit = sel(-0.08 + 0.11 * self.rand(N, RS), self.dr_hit)
            self.dr_roll = sel(0.7 + 0.7 * self.rand(N), self.dr_roll)
        # FUEL: the staged layout for this many robots, the CHUTES, and 8 preloaded per robot (with
        # no opponent, its preload goes to the NEUTRAL ZONE edges like the game does)
        ki = k - 1
        pre = self.tmpl_pre[ki]                                            # [N,BB]
        isPre = pre >= 0
        here = pres.gather(1, pre.clamp(min=0)) & isPre
        bst = torch.where(isPre, torch.where(here, HELD, FIELD), self.tmpl_st[ki])
        bown = torch.where(isPre, torch.where(here, pre, 0), self.tmpl_own[ki])
        edge = torch.stack([self.rand(N, self.BB) - 0.5, (self.rand(N, self.BB) - 0.5) * 6.8], -1)
        bp = torch.where((isPre & ~here)[..., None], edge, self.tmpl_pos[ki])
        self.bpos, self.bst, self.bown = sel(bp, self.bpos), sel(bst, self.bst), sel(bown, self.bown)
        self.stored = sel(torch.where(pres, float(self.pre), 0.0), self.stored)

    # ------------------------------------------------------------------ match clock
    def phase(self):
        t = self.t
        auto = t < self.T_AUTO
        gap = (t >= self.T_AUTO) & (t < self.T_AUTO + self.T_GAP)
        tele = (t >= self.T_AUTO + self.T_GAP) & (t < self.T_AUTO + self.T_GAP + self.T_TELE)
        post = t >= self.T_AUTO + self.T_GAP + self.T_TELE
        tt = (t - self.T_AUTO - self.T_GAP).clamp(min=0)
        return auto, gap, tele, post, tt

    def shift_index(self, tele, tt):
        s = torch.where(tt < self.T_TRANS, 0, torch.where(tt >= self.T_TELE - self.T_END, 5,
                        1 + ((tt - self.T_TRANS) / self.T_SHIFT).floor().long().clamp(0, 3)))
        return torch.where(tele, s, torch.full_like(s, -1))

    def _active_at(self, si, a):
        odd = si % 2 == 1
        fi = self.firstInactive
        return torch.where((si == 0) | (si == 5), True, torch.where(odd, a != fi, a == fi))

    def hub_active(self):
        auto, gap, tele, post, tt = self.phase()
        si = self.shift_index(tele, tt)
        return torch.stack([torch.where(auto, True, torch.where(tele, self._active_at(si, a), False)) for a in (0, 1)], 1)

    def next_change(self):
        auto, gap, tele, post, tt = self.phase()
        bounds = [self.T_TRANS + k * self.T_SHIFT for k in range(5)]
        si_now = self.shift_index(tele, tt)
        res = []
        for a in (0, 1):
            now = self._active_at(si_now, a)
            v = torch.full_like(tt, float('nan'))
            for b in reversed(bounds):
                sib = self.shift_index(tele, torch.full_like(tt, b + 0.001))
                flip = (b > tt + 1e-6) & (self._active_at(sib, a) != now)
                v = torch.where(flip, b - tt, v)
            v = torch.where(torch.isnan(v) & now, self.T_TELE - tt, v)
            out = torch.where(torch.isnan(v), 1.0, (v / self.T_SHIFT).clamp(max=1.0))
            res.append(torch.where(tele, out, 1.0))
        return torch.stack(res, 1)

    def totals(self):
        return self.score[:, :, 0] + self.score[:, :, 1]  # [N, alliance]

    def margin(self):
        """[N,RS] points margin from each robot's alliance's point of view."""
        tot = self.totals()
        al = self.alliance_idx()
        return tot.gather(1, al) - tot.gather(1, 1 - al)

    # ------------------------------------------------------------------ geometry
    def axes(self):
        c, s = self.yaw.cos(), self.yaw.sin()
        u = torch.stack([c, -s], -1)   # forward
        v = torch.stack([s, c], -1)    # right
        return u, v

    def extents(self):
        u, v = self.axes()
        hl, hw = self.spec('halfL'), self.spec('halfW')
        ex = hl * u[..., 0].abs() + hw * v[..., 0].abs()
        ez = hl * u[..., 1].abs() + hw * v[..., 1].abs()
        return ex, ez

    def in_zone(self, alliance_sign, who=None):
        """[N,RS]: any BUMPER corner of each robot in the zone of the alliance with this sign."""
        ex, _ = self.extents()
        return self.pos[..., 0] * alliance_sign - ex <= self.lineX

    def _robot_vs_rects(self, rects, mask=None):
        """Box robots vs axis-aligned boxes (SAT): push out along the least overlap, stop motion into it."""
        if rects.numel() == 0:
            return
        u, v = self.axes()
        hl, hw = self.spec('halfL'), self.spec('halfW')
        dvec = self.pos[:, :, None, :] - rects[:, :2]                     # [N,RS,K,2]
        hx, hz = rects[:, 2], rects[:, 3]
        ux, uz, vx, vz = (t[..., None] for t in (u[..., 0], u[..., 1], v[..., 0], v[..., 1]))
        hl_, hw_ = hl[..., None], hw[..., None]
        dx, dz = dvec[..., 0], dvec[..., 1]
        du, dv = dx * ux + dz * uz, dx * vx + dz * vz
        ov = torch.stack([
            hl_ * ux.abs() + hw_ * vx.abs() + hx - dx.abs(),
            hl_ * uz.abs() + hw_ * vz.abs() + hz - dz.abs(),
            hl_ + hx * ux.abs() + hz * uz.abs() - du.abs(),
            hw_ + hx * vx.abs() + hz * vz.abs() - dv.abs(),
        ], -1)                                                            # [N,RS,K,4]
        hit = (ov > 0).all(-1)
        if mask is not None:
            hit = hit & mask[..., None]
        m, j = ov.min(-1)
        ax = torch.stack([
            torch.stack([torch.ones_like(dx), torch.zeros_like(dx)], -1),
            torch.stack([torch.zeros_like(dx), torch.ones_like(dx)], -1),
            torch.stack([ux.expand_as(dx), uz.expand_as(dx)], -1),
            torch.stack([vx.expand_as(dx), vz.expand_as(dx)], -1),
        ], -2)                                                            # [N,RS,K,4,2]
        nrm = ax.gather(-2, j[..., None, None].expand(*j.shape, 1, 2)).squeeze(-2)
        sgn = torch.sign((dvec * nrm).sum(-1))
        nrm = nrm * torch.where(sgn == 0, 1.0, sgn)[..., None]
        push = torch.where(hit[..., None], nrm * m[..., None], 0.0).sum(2)
        self.pos = self.pos + push
        vn = (self.vel[:, :, None, :] * nrm).sum(-1)
        into = hit & (vn < 0)
        self.vel = self.vel - torch.where(into[..., None], vn[..., None] * nrm, 0.0).sum(2)

    def _robot_vs_robot(self):
        u, v = self.axes()
        hl, hw = self.spec('halfL'), self.spec('halfW')
        mass = self.spec('mass')
        P = self.present
        dvec = self.pos[:, :, None, :] - self.pos[:, None, :, :]           # [N,i,j,2] from j to i
        axes = torch.stack([u[:, :, None].expand(-1, -1, RS, -1), v[:, :, None].expand(-1, -1, RS, -1),
                            u[:, None].expand(-1, RS, -1, -1), v[:, None].expand(-1, RS, -1, -1)], 3)  # [N,i,j,4,2]
        proj = lambda U, V, H, W: H[..., None] * (axes * U[..., None, :]).sum(-1).abs() + W[..., None] * (axes * V[..., None, :]).sum(-1).abs()
        ri = proj(u[:, :, None].expand(-1, -1, RS, -1), v[:, :, None].expand(-1, -1, RS, -1), hl[:, :, None].expand(-1, -1, RS), hw[:, :, None].expand(-1, -1, RS))
        rj = proj(u[:, None].expand(-1, RS, -1, -1), v[:, None].expand(-1, RS, -1, -1), hl[:, None].expand(-1, RS, -1), hw[:, None].expand(-1, RS, -1))
        dist = (axes * dvec[..., None, :]).sum(-1).abs()
        ov = ri + rj - dist                                                  # [N,i,j,4]
        hit = (ov > 0).all(-1) & P[:, :, None] & P[:, None, :] & ~self._eye
        m, j = ov.min(-1)
        nrm = axes.gather(-2, j[..., None, None].expand(*j.shape, 1, 2)).squeeze(-2)
        sg = torch.sign((dvec * nrm).sum(-1))
        nrm = nrm * torch.where(sg == 0, 1.0, sg)[..., None]
        w = mass[:, None, :] / (mass[:, :, None] + mass[:, None, :])        # share of the push i takes
        self.pos = self.pos + torch.where(hit[..., None], nrm * (m * w)[..., None], 0.0).sum(2)
        rv = ((self.vel[:, :, None] - self.vel[:, None, :]) * nrm).sum(-1)
        imp = torch.where(hit & (rv < 0), rv * w, 0.0)
        self.vel = self.vel - (imp[..., None] * nrm).sum(2)

    def _walls(self):
        ex, ez = self.extents()
        lim = torch.stack([self.HL - ex, self.HW - ez], -1)
        over = (self.pos.abs() > lim) & self.present[..., None]
        self.vel = torch.where(over & (self.pos.sign() == self.vel.sign()), torch.zeros_like(self.vel), self.vel)
        p = torch.maximum(torch.minimum(self.pos, lim), -lim)
        self.pos = torch.where(self.present[..., None], p, self.pos)

    def _balls_vs_field(self, p, v, rest):
        """FUEL (circles, [N,BB,2]) out of the nearest field element (a box), bouncing; and the walls."""
        rc = self._nearest_rect(p)
        c, hh = rc[..., :2], rc[..., 2:]
        rel = p - c
        q = torch.maximum(torch.minimum(rel, hh), -hh)
        dvec = rel - q
        dist = dvec.norm(dim=-1)
        r = self.R
        hit = dist < r
        inside = dist < 1e-6
        pen = hh - rel.abs()
        ax = pen[..., 0] < pen[..., 1]
        sx, sz = rel[..., 0].sign(), rel[..., 1].sign()
        sx = torch.where(ax & (sx == 0), 1.0, sx)
        sz = torch.where(~ax & (sz == 0), 1.0, sz)
        n_in = torch.stack([torch.where(ax, sx, 0.0), torch.where(ax, 0.0, sz)], -1)
        depth = torch.where(inside, torch.where(ax, pen[..., 0], pen[..., 1]) + r, r - dist)
        n = torch.where(inside[..., None], n_in, dvec / dist.clamp(min=1e-6)[..., None])
        p = p + torch.where(hit[..., None], n * depth[..., None], 0.0)
        vn = (v * n).sum(-1)
        into = hit & (vn < 0)
        v = v + torch.where(into[..., None], -(1 + rest) * vn[..., None] * n, 0.0)
        lim = self._ball_lim
        over = p.abs() > lim
        v = torch.where(over & (p.sign() == v.sign()), -rest * v, v)
        p = torch.maximum(torch.minimum(p, lim), -lim)
        return p, v

    def _put(self, j, **vals):
        """Write per-FUEL fields at indices j ([N,M]; B = the spare slot, i.e. nobody)."""
        for name, val in vals.items():
            cur = getattr(self, name)
            if not torch.is_tensor(val):
                val = torch.full(j.shape, val, dtype=cur.dtype, device=self.dev)
            val = val.to(cur.dtype)
            if cur.dim() == 3:
                val = val.expand(*j.shape, cur.shape[-1])
                setattr(self, name, cur.scatter(1, j[..., None].expand(*j.shape, cur.shape[-1]), val))
            else:
                setattr(self, name, cur.scatter(1, j, val.expand(j.shape)))
        self.bst = torch.where(self._spare, DEAD, self.bst)

    # ------------------------------------------------------------------ one decision (0.1 s)
    def step(self, act):
        """act: [N,RS,7] network actions (own frame). Returns reward parts per robot [N,RS,7]
        (own alliance pts, their pts, FUEL this robot intaked, own inactive-HUB FUEL, margin after,
        how much the drive / turn command changed since the last decision (squared), how hard it
        turns (0-1)) and done [N]."""
        before = torch.stack([self.totals(), self.score[:, :, 2]], -1)
        auto, gap, tele, post, tt = self.phase()
        enabled = (auto | tele)[:, None] & self.present
        a3 = act[..., :3].clamp(-1, 1)
        a = torch.cat([a3, act[..., 3:]], -1)
        self.cmd = torch.where(enabled[..., None], a, 0.0)
        jerk = ((a3 - self.prev_a) ** 2).sum(-1) * enabled
        spin = a3[..., 2].abs() * enabled
        self.prev_a = torch.where(enabled[..., None], a3, self.prev_a)
        h = self.dt / self.sub
        for i in range(self.sub):
            # crowded FUEL spreads out every other substep (plenty)
            self._sub(h, enabled, i % 2 == 1 or self.sub == 1)
            self.t = self.t + h
        self._clock()
        after = torch.stack([self.totals(), self.score[:, :, 2]], -1)
        diff = after - before
        al = self.alliance_idx()
        own = diff.gather(1, al[..., None].expand(-1, -1, 2))
        oth = diff.gather(1, (1 - al)[..., None].expand(-1, -1, 2))
        rew = torch.stack([own[..., 0], oth[..., 0], self.intaked - self._prevIntaked, own[..., 1], self.margin(), jerk, spin], -1)
        self._prevIntaked = self.intaked
        done = self.t >= self.T_DONE - 1e-6
        self._commit()
        return rew, done

    def _clock(self):
        auto, gap, tele, post, tt = self.phase()
        act = self.hub_active()
        fell = self.prevActive & ~act
        self.lastDeact = torch.where(fell, self.t[:, None], self.lastDeact)
        self.prevActive = act
        start = tele & (tt < self.dt + 1e-6)
        b, r = self.score[:, 0, 0], self.score[:, 1, 0]
        fi = torch.where(b > r, 0, torch.where(r > b, 1, (self.rand(self.n) < 0.5).long()))
        self.firstInactive = torch.where(start, fi, self.firstInactive)

    # ------------------------------------------------------------------ physics substep
    def _substep(self, h, enabled, crowd):
        N, d, B, R = self.n, self.dev, self.B, self.R
        cmd = self.cmd
        s = self.sgn
        ms = self.spec('maxSpeed') * self.dr_speed
        ma = self.spec('maxAccel') * self.dr_accel
        mo, mal = self.spec('maxOmega'), self.spec('maxAlpha')
        hl, hw = self.spec('halfL'), self.spec('halfW')
        has = self.stored > 0.5
        wantShoot = enabled & ((cmd[..., 4] > 0.5) | (cmd[..., 5] > 0.5))
        wantPass = cmd[..., 5] > 0.5
        al = self.alliance_idx()
        inZ = self.in_zone(s)
        passMode = wantPass | ~inZ

        # ---- shot target and needed speed
        hubc = torch.stack([s * self.hubX, torch.zeros_like(s)], -1)
        cx = s * (-self.HL + 1.7)
        cz = torch.where((self.pos[..., 1] - (self.HW - 1.8)).abs() < (self.pos[..., 1] + (self.HW - 1.8)).abs(), self.HW - 1.8, -(self.HW - 1.8))
        passc = torch.stack([cx, cz], -1)
        tgt = torch.where(passMode[..., None], passc, hubc)
        rel = tgt - self.pos
        dist = rel.norm(dim=-1)
        psi = torch.atan2(-rel[..., 1], rel[..., 0])
        need = self._need_speed(dist, passMode)
        prespin = enabled & has & inZ
        aiming = (wantShoot | prespin) & has & ~torch.isnan(need)
        self.shot = aiming
        turret = self.spec('turret')
        fixed = ~turret
        err = wrap(psi - (self.yaw + self.spec('aimOffset')))
        w_aim = torch.minimum(mo, torch.minimum(torch.sqrt(2 * 0.7 * mal * err.abs()), 7 * err.abs())) * err.sign()
        omega_cmd = torch.where(fixed & wantShoot & aiming, w_aim, cmd[..., 2] * mo)
        relang = wrap(psi - self.yaw)
        lim = self.spec('turretRange') * math.pi / 180
        twin = self.spec('turrets') > 1.5
        goal = torch.where(twin, relang, relang.clamp(-lim, lim))
        self.turret = torch.where(turret & aiming, approach(self.turret, goal, self.spec('turretRate') * math.pi / 180 * h), self.turret)
        aimErr = torch.where(turret, wrap(psi - self.yaw - self.turret), err)
        tol = torch.where(passMode, 4 * math.pi / 180, torch.maximum(torch.full_like(dist, 0.6 * math.pi / 180), torch.atan2(torch.full_like(dist, 0.14), dist)))
        setp = torch.where(aiming, need.nan_to_num(0), torch.zeros_like(need))
        self.holdV = torch.where(setp > 0, setp, self.holdV)
        self.holdT = torch.where(setp > 0, self.t[:, None], self.holdT)
        setp = torch.where((setp == 0) & enabled & (self.t[:, None] - self.holdT < 1.5), self.holdV, setp)
        smax = self.spec('speedMax')
        e = setp.clamp(max=smax) - self.fly
        up = torch.minimum(e * (1 - math.exp(-h / 0.06)), smax / (2 * self.spec('spinTau')) * h)
        dn = torch.maximum(e * (1 - math.exp(-h / 0.06)), -(smax / 3) * h)
        self.fly = self.fly + torch.where(e > 0, up, dn)
        spinOk = (self.fly - need.nan_to_num(1e9)).abs() / need.nan_to_num(1e9).clamp(min=1e-3) < 0.025
        self.ready = aiming & spinOk & (aimErr.abs() < tol)

        # ---- drive
        vx, vz = cmd[..., 0] * s * ms, cmd[..., 1] * s * ms
        sp = torch.hypot(vx, vz)
        k = torch.where(sp > ms, ms / sp.clamp(min=1e-6), torch.ones_like(sp))
        vx, vz = vx * k, vz * k
        w = omega_cmd.clamp(-mo, mo)
        wheel = torch.hypot(vx, vz) + w.abs() * torch.hypot(hl, hw)
        k = torch.where(wheel > ms, ms / wheel.clamp(min=1e-6), torch.ones_like(wheel))
        en = enabled.float()
        vx, vz, w = vx * k * en, vz * k * en, w * k * en
        dvx, dvz = vx - self.vel[..., 0], vz - self.vel[..., 1]
        dm = torch.hypot(dvx, dvz)
        k = torch.where(dm > ma * h, ma * h / dm.clamp(min=1e-6), torch.ones_like(dm))
        self.vel = self.vel + torch.stack([dvx * k, dvz * k], -1)
        self.omega = approach(self.omega, w, mal * h)
        self.pos = self.pos + self.vel * h * self.present[..., None]
        self.yaw = wrap(self.yaw + self.omega * h)
        # collisions: field elements, TRENCH arms (tall robots), each other, the walls
        self._robot_vs_rects(self.obs_rects, self.present)
        self._robot_vs_rects(self.arms, self.present & ~self.spec('fits'))
        self._robot_vs_robot()
        self._walls()

        # ---- intake and hopper
        want = enabled & ((cmd[..., 3] > 0.5) | (cmd[..., 6] > 0.5))
        want = want | (self.spec('latched') & (self.deploy >= 1))
        self.deploy = approach(self.deploy, want.float(), h / self.spec('deployTime'))
        capIn, capOut = self.spec('capIn'), self.spec('capOut')
        hopWant = torch.where(self.spec('latched'), torch.where((self.hopper > 0.02) | (self.deploy > 0.3), 1.0, 0.0), self.deploy)
        need_h = ((self.stored - capIn) / (capOut - capIn).clamp(min=1)).clamp(0, 1)
        self.hopper = approach(self.hopper, torch.maximum(hopWant, need_h), h / 0.45)
        cap = torch.floor(capIn + (capOut - capIn) * self.hopper + 1e-6)
        jam = torch.where(self.spec('packs'), torch.floor(self.spec('capOut') * PACK_FULL), self.spec('capOut'))
        cap = torch.minimum(cap, jam)
        running = enabled & (cmd[..., 3] > 0.5) & (self.deploy > 0.85)

        # ---- FUEL on the field vs robots: swallowed by the intake, or pushed (and pushing back)
        u, v = self.axes()
        mass = self.spec('mass')
        rate = INTAKE_MAX * self.dr_intake * (self.spec('intakeRate') / 200).clamp(max=1).clamp(min=0.1)
        rate = torch.where(self.spec('intakeRate') < 40, self.spec('intakeRate'), rate)
        row = (self.spec('intakeWidth') / (2 * R)).floor().clamp(min=2)  # the rollers grip about a row
        self.tokens = torch.where(running, torch.minimum(self.tokens + rate * h, row), torch.zeros_like(self.tokens))
        room = (cap - self.stored).clamp(min=0)
        take_n = torch.minimum(room, self.tokens.floor())
        # which FUEL touch each robot's box (the intake sticks out in front): up to CONTACTS per
        # robot, listed in FUEL order (the spare slot B fills the rest)
        front = hl + self.deploy * self.spec('intakeReach')
        onf = self.bst == FIELD
        ux, uz, vx_, vz_ = u[..., 0, None], u[..., 1, None], v[..., 0, None], v[..., 1, None]
        px, pz = self.pos[..., 0, None], self.pos[..., 1, None]
        ddx = self.bpos[:, None, :, 0] - px
        ddz = self.bpos[:, None, :, 1] - pz
        lx = ddx * ux + ddz * uz
        lz = ddx * vx_ + ddz * vz_
        hl_, hw_, fr_ = hl[..., None], hw[..., None], front[..., None]
        near = (lx > -hl_ - R) & (lx < fr_ + R) & (lz.abs() < hw_ + R) & onf[:, None, :] & self.present[..., None]
        K = CONTACTS
        rank = near.to(torch.int32).cumsum(-1, dtype=torch.int32)
        dst = torch.where(near & (rank <= K), rank - 1, K).long()
        cidx = torch.full((N, RS, K + 1), B, dtype=torch.long, device=d).scatter(-1, dst, self._ar_bb.expand(N, RS, -1))[..., :K]
        flat = cidx.reshape(N, RS * K)
        cv = cidx < B
        cp = self.bpos.gather(1, flat[..., None].expand(-1, -1, 2)).view(N, RS, K, 2)
        bv = self.bvel.gather(1, flat[..., None].expand(-1, -1, 2)).view(N, RS, K, 2)
        dx, dz = cp[..., 0] - px, cp[..., 1] - pz
        lx, lz = dx * ux + dz * uz, dx * vx_ + dz * vz_
        iw = (self.spec('intakeWidth') / 2 + 0.02)[..., None]
        inI = cv & running[..., None] & (lx > hl_ - 0.04) & (lz.abs() < iw)
        # intake: up to take_n per robot, in FUEL order; a FUEL two intakes reach goes to one
        take = inI & (inI.to(torch.int32).cumsum(-1, dtype=torch.int32) <= take_n[..., None])
        rid = self._ar_rs[None, :, None].expand(N, RS, K)
        who = torch.full((N, self.BB), RS, dtype=torch.long, device=d).scatter_reduce(
            1, flat, torch.where(take, rid, RS).reshape(N, RS * K), 'amin')
        take = take & (who.gather(1, flat).view(N, RS, K) == rid)
        got = take.sum(-1).float()
        self.stored = self.stored + got
        self.tokens = self.tokens - got
        self.intaked = self.intaked + got
        taken = who < RS
        self.bst = torch.where(taken, HELD, self.bst)
        self.bown = torch.where(taken, who, self.bown)
        # FUEL in the mouth that has to wait its turn is held there, not plowed ahead (unless full)
        held = inI & ~take & (room > got)[..., None]
        push = cv & ~take & ~held
        pen_f = fr_ + R - lx
        pen_b = lx + hl_ + R
        pen_s = hw_ + R - lz.abs()
        m_f = (pen_f <= pen_b) & (pen_f <= pen_s)
        m_b = (pen_b < pen_f) & (pen_b <= pen_s)
        nlx = torch.where(m_f, lx + pen_f, torch.where(m_b, lx - pen_b, lx))
        nlz = torch.where(~m_f & ~m_b, lz + lz.sign() * pen_s, lz)
        wx, wz = px + nlx * ux + nlz * vx_, pz + nlx * uz + nlz * vz_
        om = self.omega[..., None]
        pvx = self.vel[..., 0, None] + om * (wz - pz)
        pvz = self.vel[..., 1, None] - om * (wx - px)
        # contact normal (world): out the front, the back or a side of the robot
        sz = lz.sign()
        nx = torch.where(m_f, ux, torch.where(m_b, -ux, sz * vx_))
        nz = torch.where(m_f, uz, torch.where(m_b, -uz, sz * vz_))
        # bounce off the bumper along the normal (FUEL restitution 0.45); a little slip sideways
        rx, rz = pvx - bv[..., 0], pvz - bv[..., 1]
        rn = rx * nx + rz * nz
        vn = rn.clamp(min=0) * 1.45
        dvx = (nx * vn + (rx - nx * rn) * 0.2) * push
        dvz = (nz * vn + (rz - nz * rn) * 0.2) * push
        delta = torch.stack([(wx - cp[..., 0]) * push, (wz - cp[..., 1]) * push, dvx, dvz], -1)   # [N,RS,K,4]
        acc = torch.zeros(N, self.BB, 4, device=d).scatter_add(1, flat[..., None].expand(-1, -1, 4), delta.view(N, RS * K, 4))
        self.bpos = self.bpos + acc[..., :2]
        self.bvel = self.bvel + acc[..., 2:]
        # the FUEL it shoves takes momentum from the robot
        self.vel = self.vel - torch.stack([dvx.sum(-1), dvz.sum(-1)], -1) * (0.215 / mass[..., None])

        # ---- shoot / pass: feed while ready
        period = 1.0 / self.spec('bps')
        self.feedT = torch.minimum(self.feedT + h, period * 1.5)
        fire = wantShoot & self.ready & has & (self.feedT >= period)
        self.feeding = wantShoot & self.ready & has
        self.feedT = torch.where(fire, self.feedT - period, self.feedT)
        mine = torch.where(self.bst == HELD, self.bown, -1)[:, None, :] == self._ar_rs[None, :, None]   # [N,RS,BB]
        nheld = mine.sum(-1)
        fire = fire & (nheld > 0)
        first = mine.to(torch.uint8).argmax(-1)
        last = (mine.to(torch.int16) * self._ar_bb16).argmax(-1)
        spd = self.vel.norm(dim=-1)
        fr = (spd / ms.clamp(min=0.1)).clamp(0, 1) * 3
        i0 = fr.floor().long().clamp(max=2)
        acc = self.shotAcc[self.rtype]
        a0, a1 = acc.gather(-1, i0[..., None]).squeeze(-1), acc.gather(-1, (i0 + 1)[..., None]).squeeze(-1)
        p_hit = (a0 + (a1 - a0) * (fr - i0) + self.dr_hit).clamp(0.4, 0.99)
        hitb = self.rand(N, RS) < p_hit
        kind = torch.where(passMode, FLY_LAND, torch.where(hitb, FLY_HIT, FLY_MISS))
        tf = torch.where(passMode, 0.5 + 0.1 * dist, 0.35 + 0.12 * dist) * (0.9 + 0.2 * self.rand(N, RS))
        land = torch.where(passMode[..., None], tgt + self.randn(N, RS, 2) * 0.4, hubc)
        self._put(torch.where(fire, first, B), bst=FLY, bkind=kind, btim=tf, bland=land, bown=al, bpos=self.pos)
        ff = fire.float()
        self.stored = self.stored - ff
        self.shots = self.shots + ff
        self.passes = self.passes + ff * passMode.float()
        # ---- outtake: spit FUEL out the front (the last one in; the first may just have been shot)
        out = enabled & (cmd[..., 6] > 0.5) & has & (self.deploy > 0.85)
        go = out & (self.rand(N, RS) < h / 0.08) & (nheld - fire.long() >= 1)
        hl2 = (hl + 0.25)[..., None]
        self._put(torch.where(go, last, B), bst=FIELD, bpos=self.pos + u * hl2, bvel=self.vel + u * 2.8)
        self.stored = self.stored - go.float()

        # ---- human players
        act = self.hub_active()
        self.hpCool = (self.hpCool - h).clamp(min=0)
        for a in (0, 1):
            ch = (self.bst == CHUTE) & (self.bown == a)
            go = act[:, a] & (self.hpCool[:, a] <= 0) & ch.any(1)
            j = torch.where(go, ch.to(torch.uint8).argmax(1), B)[:, None]
            hitb = self.rand(N, 1) < 0.5
            self._put(j, bst=FLY, bkind=torch.where(hitb, FLY_HIT, FLY_MISS), btim=1.2, bland=self._hubw[a].expand(N, 1, 2), bown=a)
            self.hpCool = torch.where(go[:, None] & (self._ar_rs[:2] == a), 1.05, self.hpCool)

        # ---- FUEL in flight lands / enters the HUB; HUB processing and exits
        fly = self.bst == FLY
        self.btim = torch.where(fly | (self.bst == HUBQ), self.btim - h, self.btim)
        self._arrive(fly & (self.btim <= 0), act)
        self._exit_hub((self.bst == HUBQ) & (self.btim <= 0))

        # ---- crowded FUEL spreads out (FUEL can't stack up in one spot)
        if crowd:
            self._crowding(h)

        # ---- FUEL rolls: carpet resistance, walls, field elements
        onf = self.bst == FIELD
        sp = self.bvel.norm(dim=-1)
        dec = 0.35 * self.dr_roll[:, None] * h
        k = torch.where(sp <= dec + 0.01, torch.zeros_like(sp), (sp - dec) / sp.clamp(min=1e-6))
        self.bvel = self.bvel * k[..., None]
        moving = (onf & (sp > 0.01))[..., None]
        p, v = self._balls_vs_field(self.bpos + self.bvel * h, self.bvel, 0.45)
        self.bpos = torch.where(moving, p, self.bpos)
        self.bvel = torch.where(moving, v, self.bvel)

    def _crowding(self, h):
        """FUEL can't stack: FUEL sharing a radius-sized cell overlap (FUEL a diameter apart, like the
        staged rows, never share one); each such pair is pushed apart along the line between them,
        as far as they overlap."""
        onf = self.bst == FIELD
        c = self.R
        gx = ((self.bpos[..., 0] + self.HL) / c).long()
        gz = ((self.bpos[..., 1] + self.HW) / c).long()
        cell = torch.where(onf, gx * 100000 + gz, -1 - self._ar_bb[None])
        srt, order = cell.sort(1)
        same = srt[:, 1:] == srt[:, :-1]                          # neighbours in sorted order share a cell
        ia, ib = order[:, :-1], order[:, 1:]
        pa = self.bpos.gather(1, ia[..., None].expand(-1, -1, 2))
        pb = self.bpos.gather(1, ib[..., None].expand(-1, -1, 2))
        dvec = pa - pb
        dist = dvec.norm(dim=-1)
        rnd = self.randn(*dist.shape, 2)
        nrm = torch.where((dist > 1e-4)[..., None], dvec / dist.clamp(min=1e-4)[..., None], rnd / rnd.norm(dim=-1, keepdim=True).clamp(min=1e-6))
        ov = (2 * self.R - dist).clamp(min=0) * same
        move = nrm * (ov / 2)[..., None]                           # each moves half the overlap
        dp = torch.zeros_like(self.bpos).scatter_add(1, ia[..., None].expand(-1, -1, 2), move).scatter_add(1, ib[..., None].expand(-1, -1, 2), -move)
        self.bpos = torch.where(onf[..., None], self.bpos + dp, self.bpos)

    def _need_speed(self, dist, passMode):
        def look(t, d):
            i = (d / 0.5).clamp(0, t.shape[-1] - 1.001)
            i0 = i.floor().long()
            fr = i - i0
            v0 = t.gather(-1, i0[..., None]).squeeze(-1)
            v1 = t.gather(-1, (i0 + 1).clamp(max=t.shape[-1] - 1)[..., None]).squeeze(-1)
            return torch.where(d / 0.5 > t.shape[-1] - 1, torch.nan, v0 + (v1 - v0) * fr)
        return torch.where(passMode, look(self.passV[self.rtype], dist), look(self.hubV[self.rtype], dist))

    def _arrive(self, arrive, act):
        auto, gap, tele, post, tt = self.phase()
        N, BB = self.n, self.BB
        k = self.bkind
        a = self.bown.clamp(0, 1)  # alliance index for FUEL in flight (held FUEL stores its robot)
        hit = arrive & (k == FLY_HIT)
        actb = act.gather(1, a)
        graceb = (self.t[:, None] - self.lastDeact.gather(1, a)) <= self.grace
        counts = hit & (actb | graceb)
        autoP = auto[:, None] | (gap[:, None] & graceb)
        add = [torch.stack([(counts & (a == alx) & autoP).sum(1), (counts & (a == alx) & ~autoP).sum(1),
                            (hit & ~(actb | graceb) & (a == alx)).sum(1)], -1) for alx in (0, 1)]
        self.score = self.score + torch.stack(add, 1).float()
        self.bst = torch.where(hit, HUBQ, self.bst)
        self.btim = torch.where(hit, 0.45 + 0.85 * self.rand(N, BB), self.btim)
        miss = arrive & (k == FLY_MISS)
        ang = self.rand(N, BB) * 2 * math.pi
        r = self.hs + 0.15 + 0.8 * self.rand(N, BB)
        dirv = torch.stack([ang.cos(), ang.sin()], -1)
        self.bpos = torch.where(miss[..., None], self.bland + dirv * r[..., None], self.bpos)
        self.bvel = torch.where(miss[..., None], dirv * 0.8, self.bvel)
        self.bst = torch.where(miss, FIELD, self.bst)
        land = arrive & (k == FLY_LAND)
        self.bpos = torch.where(land[..., None], self.bland, self.bpos)
        self.bvel = torch.where(land[..., None], self.randn(N, BB, 2) * 0.4, self.bvel)
        self.bst = torch.where(land, FIELD, self.bst)

    def _exit_hub(self, ex):
        N, BB = self.n, self.BB
        a = self.bown.clamp(0, 1)
        s = torch.where(a == 0, 1.0, -1.0)
        off = (self.rand(N, BB) - 0.5) * (self.exitW - 2 * self.R)
        ang = off / (self.exitW / 2) * 0.5 + (self.rand(N, BB) - 0.5) * 0.8
        sp = self.exitSpeed[0] + (self.exitSpeed[1] - self.exitSpeed[0]) * self.rand(N, BB)
        vx, vz = s * ang.cos() * sp, s * ang.sin() * sp
        x = torch.where(a == 0, self.hubX, -self.hubX) + s * (self.hs + self.R + 0.03) + vx * 0.4
        z = s * off + vz * 0.4
        self.bpos = torch.where(ex[..., None], torch.stack([x, z], -1), self.bpos)
        self.bvel = torch.where(ex[..., None], torch.stack([vx, vz], -1) * 0.8, self.bvel)
        self.bst = torch.where(ex, FIELD, self.bst)

    # ------------------------------------------------------------------ observation (= js/nn/obs.js)
    def obs(self):
        """[N,RS,D] observation of each robot, built like buildObs() in js/nn/obs.js."""
        N, d = self.n, self.dev
        s = self.sgn
        parts = []
        P = lambda *xs: parts.append(torch.stack(xs, -1))
        x, z = s * self.pos[..., 0], s * self.pos[..., 1]
        yaw = self.yaw + torch.where(s > 0, 0.0, math.pi)
        c, sn = yaw.cos(), yaw.sin()
        turret = self.spec('turret')
        cap = self.spec('capMax')
        capNow = torch.floor(self.spec('capIn') + (self.spec('capOut') - self.spec('capIn')) * self.hopper + 1e-6)
        inZ = self.in_zone(s)
        P(x / self.HL, z / self.HW, c, sn, s * self.vel[..., 0] / 5, s * self.vel[..., 1] / 5, self.omega / 10,
          self.stored / 60, self.stored / cap, capNow / cap, self.deploy, self.hopper, self.fly / self.spec('speedMax'),
          self.ready.float(), self.shot.float(), inZ.float(), self.feeding.float(),
          torch.where(turret, self.turret / math.pi, torch.zeros_like(x)))
        parts.append((self.rtype[..., None] == self._ar_nr).float())
        P(self.spec('maxSpeed') / 5, self.spec('bps') / 40, turret.float())
        for own in (True, False):
            hx = torch.full_like(x, self.hubX) * (1 if own else -1)
            dx, dz = hx - x, -z
            lx, lz = dx * c - dz * sn, dx * sn + dz * c
            P(lx / 8, lz / 8, torch.hypot(lx, lz) / 8)
        P((self.lineX - x) / self.HL)
        auto, gap, tele, post, tt = self.phase()
        left = torch.where(auto, self.TOTAL - self.t, torch.where(tele, self.T_TELE - tt, torch.where(gap, torch.full_like(tt, self.T_TELE), torch.zeros_like(tt))))
        ph = torch.where(auto, (self.T_AUTO - self.t) / self.T_AUTO, torch.where(tele, (self.T_TELE - tt) / self.T_TELE, torch.zeros_like(tt)))
        si = self.shift_index(tele, tt)
        parts.append(torch.stack([auto.float(), tele.float(), left / self.TOTAL, ph], -1)[:, None].expand(N, RS, 4))
        parts.append(torch.stack([(si == k).float() for k in range(6)], -1)[:, None].expand(N, RS, 6))
        act = self.hub_active().float()
        al = self.alliance_idx()
        nc = self.next_change()
        tot = self.totals()
        mine, theirs = tot.gather(1, al), tot.gather(1, 1 - al)
        P(act.gather(1, al), act.gather(1, 1 - al), nc.gather(1, al), nc.gather(1, 1 - al), torch.tanh((mine - theirs) / 100), mine / 300)
        # the other robots, as seen by each robot: [N, observer, target, 13+NR]
        blk = self._robot_blocks(x, z, c, sn, s)
        d2 = ((self.pos[:, :, None] - self.pos[:, None]) ** 2).sum(-1)
        same = al[:, :, None] == al[:, None, :]
        eye = self._eye
        pres_t = self.present[:, None, :].expand(N, RS, RS)
        big = torch.full_like(d2, 1e9)
        foes_d = torch.where(~same & pres_t, d2, big)
        mates_d = torch.where(same & pres_t & ~eye, d2, big)
        fd, fi = foes_d.sort(-1)
        md, mi = mates_d.sort(-1)
        def pick(idx, dd, k):
            b = blk.gather(2, idx[..., k:k + 1, None].expand(-1, -1, -1, blk.shape[-1])).squeeze(2)
            return b * (dd[..., k:k + 1] < 1e8).float()
        parts.append(pick(fi, fd, 0))
        parts.append(self._rays(yaw, s) / 4.0)
        parts.append(self._fuel_obs(x, z, c, sn, s))
        parts += [pick(mi, md, 0), pick(mi, md, 1), pick(fi, fd, 1), pick(fi, fd, 2)]
        if self.NEW:
            # (v4) which robot each is, for robots added since v3: itself, the nearest opponent, the
            # teammates, the 2nd/3rd opponents
            rt = self.rtype[:, None, :].expand(N, RS, RS)
            def new_oh(idx, dd, k):
                r = rt.gather(2, idx[..., k:k + 1]).squeeze(2)
                return (r[..., None] == self._ar_new).float() * (dd[..., k:k + 1] < 1e8).float()
            parts += [(self.rtype[..., None] == self._ar_new).float(),
                      new_oh(fi, fd, 0), new_oh(mi, md, 0), new_oh(mi, md, 1), new_oh(fi, fd, 1), new_oh(fi, fd, 2)]
        return torch.cat(parts, -1)

    def _robot_blocks(self, x, z, c, sn, s):
        """Every robot's description as seen by every other robot: [N,obs,target,13+NR]."""
        N = self.n
        sO = s[:, :, None]                          # observer's frame
        fx = sO * self.pos[:, None, :, 0]
        fz = sO * self.pos[:, None, :, 1]
        dx, dz = fx - x[..., None], fz - z[..., None]
        lx = dx * c[..., None] - dz * sn[..., None]
        lz = dx * sn[..., None] + dz * c[..., None]
        fyaw = self.yaw[:, None, :] + torch.where(sO > 0, 0.0, math.pi)
        ex, _ = self.extents()
        inMine = sO * self.pos[:, None, :, 0] - ex[:, None, :] <= self.lineX
        inOwn = (self.sgn * self.pos[..., 0] - ex <= self.lineX)[:, None, :].expand(N, RS, RS)
        one = torch.ones_like(fx)
        feat = torch.stack([one, fx / self.HL, fz / self.HW, lx / 8, lz / 8, torch.hypot(lx, lz) / 8,
                            sO * self.vel[:, None, :, 0] / 5, sO * self.vel[:, None, :, 1] / 5, fyaw.cos(), fyaw.sin(),
                            self.stored[:, None, :].expand(N, RS, RS) / 60], -1)
        oh = (self.rtype[..., None] == self._ar_nr).float()[:, None].expand(N, RS, RS, self.NOH)
        return torch.cat([feat, oh, torch.stack([inMine.float(), inOwn.float()], -1)], -1)

    def _rays(self, yaw_own, s):
        k = torch.arange(16, device=self.dev).float() / 16 * 2 * math.pi
        ang = yaw_own[..., None] + k
        dx, dz = ang.cos() * s[..., None], -ang.sin() * s[..., None]
        px, pz = self.pos[..., 0, None], self.pos[..., 1, None]
        eps = 1e-6
        tx = torch.where(dx > eps, (self.HL - px) / dx.clamp(min=eps), torch.where(dx < -eps, (-self.HL - px) / dx.clamp(max=-eps), torch.full_like(dx, 1e9)))
        tz = torch.where(dz > eps, (self.HW - pz) / dz.clamp(min=eps), torch.where(dz < -eps, (-self.HW - pz) / dz.clamp(max=-eps), torch.full_like(dz, 1e9)))
        best = torch.minimum(torch.full_like(dx, 4.0), torch.minimum(tx, tz))
        R = self.obs_rects
        def slab(p, dd, lo, hi):
            pe, de = p[..., None], dd[..., None]
            par = de.abs() < 1e-9
            ta = (lo - pe) / torch.where(par, torch.ones_like(de), de)
            tb = (hi - pe) / torch.where(par, torch.ones_like(de), de)
            t0, t1 = torch.minimum(ta, tb), torch.maximum(ta, tb)
            outside = par & ((pe < lo) | (pe > hi))
            return torch.where(par, -1e9, t0), torch.where(par, 1e9, t1), outside
        a0, a1, ao = slab(px, dx, R[:, 0] - R[:, 2], R[:, 0] + R[:, 2])
        b0, b1, bo = slab(pz, dz, R[:, 1] - R[:, 3], R[:, 1] + R[:, 3])
        t0 = torch.maximum(torch.maximum(a0, b0), torch.zeros_like(a0))
        t1 = torch.minimum(torch.minimum(a1, b1), best[..., None])
        hit = (t0 <= t1) & ~ao & ~bo
        best = torch.minimum(best, torch.where(hit, t0, torch.full_like(t0, 1e9)).min(-1).values)
        return best.clamp(min=0)

    def _fuel_obs(self, x, z, c, sn, s):
        N, d = self.n, self.dev
        onf = (self.bst == FIELD)[:, None, :].expand(N, RS, self.BB)
        bx = s[..., None] * self.bpos[:, None, :, 0]
        bz = s[..., None] * self.bpos[:, None, :, 1]
        dx, dz = bx - x[..., None], bz - z[..., None]
        lx = dx * c[..., None] - dz * sn[..., None]
        lz = dx * sn[..., None] + dz * c[..., None]
        EN, EC = 12, 0.4
        half = EN * EC / 2
        ine = onf & (lx > -half) & (lx < half) & (lz > -half) & (lz < half)
        ex = ((lx + half) / EC).floor().long().clamp(0, EN - 1)
        ez = ((lz + half) / EC).floor().long().clamp(0, EN - 1)
        ei = torch.where(ine, ex * EN + ez, torch.full_like(ex, EN * EN))
        ego = torch.zeros(N, RS, EN * EN + 1, device=d).scatter_add_(-1, ei, torch.ones_like(lx))[..., :EN * EN]
        ego = (ego / 4).clamp(max=1)
        # the whole-field FUEL map and zone counts are the same for every robot of an ALLIANCE:
        # build them once per alliance (blue's frame, then red's: the field rotated 180 deg)
        GX, GZ = 16, 8
        on1 = self.bst == FIELD
        grids, zones = [], []
        for sgn_ in (1.0, -1.0):
            ax, az = sgn_ * self.bpos[..., 0], sgn_ * self.bpos[..., 1]
            gx = ((ax + self.HL) / (2 * self.HL) * GX).floor().long()
            gz = ((az + self.HW) / (2 * self.HW) * GZ).floor().long()
            ing = on1 & (gx >= 0) & (gx < GX) & (gz >= 0) & (gz < GZ)
            gi = torch.where(ing, gx.clamp(0, GX - 1) * GZ + gz.clamp(0, GZ - 1), torch.full_like(gx, GX * GZ))
            grids.append(torch.zeros(N, GX * GZ + 1, device=d).scatter_add_(-1, gi, torch.ones_like(ax))[:, :GX * GZ])
            line = self.lineX
            zones.append(torch.stack([(on1 & (ax < line)).sum(-1).float() / 50, (on1 & (ax >= line) & (ax <= -line)).sum(-1).float() / 400,
                                      (on1 & (ax > -line)).sum(-1).float() / 50], -1))
        al = (s < 0).long()                                              # [N,RS]
        G = torch.stack(grids, 1)                                        # [N,2,128]
        grid = torch.log1p(G.gather(1, al[..., None].expand(-1, -1, GX * GZ))) / math.log1p(20)
        Z3 = torch.stack(zones, 1).gather(1, al[..., None].expand(-1, -1, 3))
        d2 = torch.where(onf, lx * lx + lz * lz, torch.full_like(lx, 1e9))
        vals, ii = torch.topk(d2, 8, dim=-1, largest=False)
        nl, nz = lx.gather(-1, ii), lz.gather(-1, ii)
        ok = vals < 1e8
        near = torch.stack([torch.where(ok, (nl / 4).clamp(-1, 1), torch.ones_like(nl)), torch.where(ok, (nz / 4).clamp(-1, 1), torch.zeros_like(nz))], -1).reshape(N, RS, 16)
        return torch.cat([ego, grid, near, Z3], -1)

    # ------------------------------------------------------------------ scripted robots
    def bot_actions(self, sl=slice(None)):
        """Simple scripted robots: [n,RS,7] actions (own frame) for the matches in slice sl. Most
        score (collect the nearest FUEL, score in their zone while their HUB is active); a few
        defend (get between the other ALLIANCE's most loaded robot and its HUB)."""
        d = self.dev
        s = self.sgn[sl]
        pos = self.pos[sl]
        n = s.shape[0]
        x, z = s * pos[..., 0], s * pos[..., 1]
        yaw = self.yaw[sl] + torch.where(s > 0, 0.0, math.pi)
        al = self.alliance_idx()[sl]
        act = self.hub_active()[sl].gather(1, al)
        nc = self.next_change()[sl].gather(1, al) * self.T_SHIFT
        cap = self.spec('capMax')[sl]
        n_ = self.stored[sl]
        present = self.present[sl]
        auto, gap, tele, post, tt = (q[sl] for q in self.phase())
        go_score = (n_ >= 0.6 * cap) | ((n_ >= 4) & (act | (nc < 4))) | (auto[:, None] & (n_ > 0) & (self.t[sl][:, None] > 12))
        # the FUEL to go for: skip FUEL tucked against a wall or a field element (no path planning
        # here); the field is point-symmetric, so this doesn't depend on whose frame it's seen in
        onf = self.bst[sl] == FIELD
        wb = self.bpos[sl]                                               # [n,BB,2] world
        rc = self._nearest_rect(wb)
        gapb = torch.maximum((wb[..., 0] - rc[..., 0]).abs() - rc[..., 2], (wb[..., 1] - rc[..., 1]).abs() - rc[..., 3])
        ok = onf & (gapb > 0.35) & (wb[..., 0].abs() < self.HL - 0.4) & (wb[..., 1].abs() < self.HW - 0.4)
        bx = s[..., None] * wb[:, None, :, 0]
        bz = s[..., None] * wb[:, None, :, 1]
        okr = ok[:, None, :].expand_as(bx)
        d2 = torch.where(okr, (bx - x[..., None]) ** 2 + (bz - z[..., None]) ** 2, torch.full_like(bx, 1e9))
        # prefer FUEL with company (dense patches), not a lone ball it'll knock away
        GX, GZ = 16, 8
        gi = (((bx + self.HL) / (2 * self.HL) * GX).long().clamp(0, GX - 1) * GZ + ((bz + self.HW) / (2 * self.HW) * GZ).long().clamp(0, GZ - 1))
        dens = torch.zeros(n, RS, GX * GZ, device=d).scatter_add(-1, gi, okr.float())
        jj = (d2 - 0.6 * dens.gather(-1, gi).clamp(max=12)).argmin(-1)
        fx = bx.gather(-1, jj[..., None]).squeeze(-1)
        fz = bz.gather(-1, jj[..., None]).squeeze(-1)
        tx = torch.where(go_score, torch.full_like(x, self.lineX - 1.2), fx)
        tz = torch.where(go_score, torch.where(z > 0, 1.7, -1.7), fz)
        # defenders: between the most loaded opponent and its HUB (own frame: their HUB is at -hubX)
        foe = (al[:, :, None] != al[:, None, :]) & present[:, None, :]
        foe_load = torch.where(foe, n_[:, None, :].expand(n, RS, RS), -1.0)
        tgt = foe_load.argmax(-1)
        ox = s * pos[..., 0].gather(1, tgt)
        oz = s * pos[..., 1].gather(1, tgt)
        dfx, dfz = ox + (-self.hubX - ox) * 0.35, oz * 0.65
        defend = self.bot_def[sl] & tele[:, None]
        tx, tz = torch.where(defend, dfx, tx), torch.where(defend, dfz, tz)
        dx, dz = tx - x, tz - z
        dist = torch.hypot(dx, dz).clamp(min=1e-3)
        vx, vz = dx / dist, dz / dist
        Rr = self.obs_rects
        rx = (x[..., None] - Rr[:, 0]).abs() - Rr[:, 2]
        rz = (z[..., None] - Rr[:, 1]).abs() - Rr[:, 3]
        g = torch.maximum(rx, rz)
        close = g < 0.9
        px = torch.where(close, (x[..., None] - Rr[:, 0]).sign() * (rx > rz).float() / g.clamp(min=0.05), 0.0).sum(-1)
        pz = torch.where(close, (z[..., None] - Rr[:, 1]).sign() * (rz >= rx).float() / g.clamp(min=0.05), 0.0).sum(-1)
        vx, vz = vx + 0.06 * px, vz + 0.06 * pz
        # stuck (asking to move, not moving): sidestep
        stuck = (self.vel[sl].norm(dim=-1) < 0.15) & (dist > 0.6) & present
        side = torch.where(self.bot_skill[sl] > 0.875, 1.0, -1.0)
        vx, vz = torch.where(stuck, vx - side * vz * 1.5, vx), torch.where(stuck, vz + side * vx * 1.5, vz)
        nv = torch.hypot(vx, vz).clamp(min=1e-3)
        sp = torch.where(go_score | defend, (dist / 0.8).clamp(0.0, 1.0), (dist / 1.5).clamp(0.4, 1.0)) * self.bot_skill[sl]
        want = torch.atan2(-dz, dx)
        collecting = ~go_score & ~defend
        inZ = self.in_zone(self.sgn)[sl]
        return torch.stack([vx / nv * sp, vz / nv * sp, (wrap(want - yaw) * 2).clamp(-1, 1) * (~go_score).float(), collecting.float(),
                            (go_score & ~defend & inZ & (act | (nc < 1.0))).float(), torch.zeros_like(x), torch.zeros_like(x)], -1)

    # ------------------------------------------------------------------ live view
    def live_pack(self, idx):
        """A bird's-eye snapshot of the matches idx (long tensor) as one float tensor [len(idx), F],
        made on the GPU without waiting; live_frames() turns a stack of them into dashboard frames."""
        L = idx.shape[0]
        tot = self.totals()[idx]
        act = self.hub_active()[idx].float()
        head = torch.stack([self.t[idx], tot[:, 0], tot[:, 1], act[:, 0], act[:, 1], self.opp_kind[idx].float()], -1)
        rob = torch.stack([self.pos[idx][..., 0], self.pos[idx][..., 1], self.yaw[idx], (self.sgn[idx] < 0).float(), self.rtype[idx].float(),
                           self.stored[idx], self.learning()[idx].float(), self.present[idx].float()], -1).reshape(L, RS * LIVE_ROBOT)
        balls = torch.where((self.bst[idx] == FIELD)[..., None], self.bpos[idx], torch.nan).reshape(L, self.BB * 2)
        return torch.cat([head, rob, balls], -1)

    def live_frames(self, packs):
        """packs: numpy [steps, matches, F] from live_pack() -> one frame (plain Python data) per step."""
        out = []
        for st in packs:
            envs = []
            for e in st:
                rob = e[LIVE_HEAD:LIVE_HEAD + RS * LIVE_ROBOT].reshape(RS, LIVE_ROBOT)
                b = e[LIVE_HEAD + RS * LIVE_ROBOT:].reshape(-1, 2)
                b = b[np.isfinite(b[:, 0])]
                envs.append({
                    't': round(float(e[0]), 2), 'score': [int(e[1]), int(e[2])], 'active': [bool(e[3] > 0.5), bool(e[4] > 0.5)], 'opp': int(e[5]),
                    'robots': [[round(float(r[0]), 3), round(float(r[1]), 3), round(float(r[2]), 3), int(r[3]), self.order[int(r[4])], int(r[5]), int(r[6])]
                               for r in rob if r[7] > 0.5],
                    'balls': np.clip(np.round(b * 100), -32000, 32000).astype('<i2').tobytes(),
                })
            out.append({'envs': envs})
        return out

    # ------------------------------------------------------------------ NVIDIA speedups
    def compile(self, log=print):
        """Fuse the physics, the reset and the observation into compiled GPU kernels (torch.compile,
        needs Triton). Each part that fails to compile stays as normal PyTorch. Returns True if any
        part compiled."""
        try:
            import torch._inductor.config as ic
            ic.fallback_random = True  # random numbers from PyTorch's own generator (a CUDA graph records it)
        except Exception:  # noqa: BLE001
            pass
        enabled = self.present.clone()
        none = torch.zeros(self.n, dtype=torch.bool, device=self.dev)
        done = []

        def attempt(name, fn, test=None):
            try:
                c = torch.compile(fn, dynamic=False)
                if test:
                    test(c)
            except Exception as e:  # noqa: BLE001 - any compiler problem: keep that part as it is
                self._rebind()
                log(f'torch.compile could not compile the {name} ({str(e).splitlines()[0][:120]}); it runs uncompiled')
                return fn
            done.append(name)
            box = [c]

            def call(*a, **k):
                try:
                    return box[0](*a, **k)
                except Exception as e:  # noqa: BLE001 - e.g. a later recompile failing: go uncompiled
                    if box[0] is fn:
                        raise
                    log(f'torch.compile failed on the {name} ({str(e).splitlines()[0][:120]}); it runs uncompiled from now on')
                    box[0] = fn
                    return fn(*a, **k)
            return call

        def t_sub(c):
            c(self.dt / self.sub, enabled, False)
            c(self.dt / self.sub, enabled, True)
            self._commit()

        def t_reset(c):
            c(none)
            self._commit()

        with torch.no_grad():  # as training runs it (a different grad mode would compile again)
            self._sub = attempt('physics', self._substep, t_sub)
            self._reset_fn = attempt('reset', self._reset, t_reset)
            self.obs = attempt('observation', self.obs, lambda c: c())
            self.bot_actions = attempt('scripted robots', self.bot_actions)
        return bool(done)
