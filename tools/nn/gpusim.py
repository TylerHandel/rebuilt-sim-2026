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

Robot slots: 0-2 are alliance A, 3-5 alliance B. A match has 1-3 robots per alliance.
Robot specs, the field and the FUEL layout come from tools/nn/gpusim-params.json, written by
tools/nn/gpusim-export.mjs from the real game.
"""
import json
import math
from pathlib import Path

import torch
import torch.nn.functional as Fn

PARAMS = Path(__file__).with_name('gpusim-params.json')

FIELD, HELD, FLY, HUBQ, CHUTE = 0, 1, 2, 3, 4
FLY_HIT, FLY_MISS, FLY_LAND = 0, 1, 2
OPP_NONE, OPP_BOT, OPP_SNAP, OPP_SELF = 0, 1, 2, 3
SUB = 4      # physics substeps per decision
RS = 6       # robot slots per match
TEAM = torch.tensor([0, 0, 0, 1, 1, 1])
PACK_FULL = 0.8  # packing hoppers jam at about this fraction of their listed capacity
INTAKE_MAX = 70.0  # FUEL/s a real intake swallows driving through a pile (tools/intake-test.mjs)


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
        self.gen = torch.Generator(device=self.dev)
        self.gen.manual_seed(seed)
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
        self.starts = torch.tensor([[P['robots'][r]['starts'][k] for k in P['startOrder']] for r in self.order], device=d)  # [NR,5,2] blue
        self.allowed = torch.tensor([self.order.index(r) for r in (robots or self.order)], device=d)

        # ---- field geometry
        self.obs_rects = torch.tensor([[o['x'], o['z'], o['hx'], o['hz']] for o in P['obstacles']], device=d)
        self.arms = torch.tensor([[a['x'], a['z'], a['hx'], a['hz']] for a in P['trenchArms']], device=d)
        # the real game stages fewer FUEL in the NEUTRAL ZONE the more robots preload: 1v1/2v2/3v3
        self.layouts = {int(k): torch.tensor(v, device=d) for k, v in P['fuel']['layouts'].items()}
        self.chuteN = P['fuel']['perChute']
        self.pre = P['fuel']['preload']
        self.B = P['fuel']['total']
        thr = P['outposts']
        self.hpThrow = torch.tensor([[thr['blue']['throw'][0], thr['blue']['throw'][2]], [thr['red']['throw'][0], thr['red']['throw'][2]]], device=d)
        self.exitW = P['hub']['exitWidth']
        self.exitSpeed = (1.0, 2.2)

        self._alloc()
        self.opp_probs = torch.tensor([0.3, 0.7, 0.0, 0.0], device=d)
        self.reset(torch.ones(n, dtype=torch.bool, device=d))

    # ------------------------------------------------------------------ helpers
    def rand(self, *shape):
        return torch.rand(*shape, device=self.dev, generator=self.gen)

    def randn(self, *shape):
        return torch.randn(*shape, device=self.dev, generator=self.gen)

    def spec(self, k):
        return self.S[k][self.rtype]  # [N,RS]

    def _alloc(self):
        n, d, B = self.n, self.dev, self.B
        z = lambda *s: torch.zeros(*s, device=d)
        zb = lambda *s: torch.zeros(*s, dtype=torch.bool, device=d)
        self.rtype = torch.zeros(n, RS, dtype=torch.long, device=d)
        self.sgn = z(n, RS)
        self.present = zb(n, RS)
        self.k = torch.ones(n, dtype=torch.long, device=d)      # robots per alliance
        self.pos, self.vel = z(n, RS, 2), z(n, RS, 2)
        self.yaw, self.omega = z(n, RS), z(n, RS)
        self.deploy, self.hopper, self.fly, self.turret = z(n, RS), z(n, RS), z(n, RS), z(n, RS)
        self.stored, self.tokens, self.feedT = z(n, RS), z(n, RS), z(n, RS)
        self.ready, self.shot, self.feeding = zb(n, RS), zb(n, RS), zb(n, RS)
        self.holdV, self.holdT = z(n, RS), z(n, RS) - 99
        self.cmd = z(n, RS, 7)
        self.intaked, self.shots, self.passes, self._prevIntaked = z(n, RS), z(n, RS), z(n, RS), z(n, RS)
        self.t = z(n)
        self.firstInactive = torch.zeros(n, dtype=torch.long, device=d)
        self.prevActive = zb(n, 2)
        self.lastDeact = z(n, 2) - 1e9
        self.score = z(n, 2, 3)            # per alliance (0 blue, 1 red): autoFuel, teleFuel, inactiveFuel
        self.hpCool = z(n, 2)
        self.opp_kind = torch.zeros(n, dtype=torch.long, device=d)
        self.bot_skill = z(n, RS) + 1
        self.bot_def = zb(n, RS)
        # domain randomization (multipliers / offsets per robot and per match)
        self.dr_speed, self.dr_accel, self.dr_intake = z(n, RS) + 1, z(n, RS) + 1, z(n, RS) + 1
        self.dr_hit, self.dr_roll = z(n, RS), z(n) + 1
        self.bpos, self.bvel = z(n, B, 2), z(n, B, 2)
        self.bst = torch.zeros(n, B, dtype=torch.long, device=d)
        self.bown = torch.zeros(n, B, dtype=torch.long, device=d)
        self.btim = z(n, B)
        self.bland = z(n, B, 2)
        self.bkind = torch.zeros(n, B, dtype=torch.long, device=d)

    def alliance_idx(self):
        return (self.sgn < 0).long()  # [N,RS] 0 blue 1 red

    def learning(self):
        """[N,RS]: robots driven by the policy being trained (alliance A, and B in self-play)."""
        return self.present & ((self.team == 0) | (self.opp_kind == OPP_SELF)[:, None])

    # ------------------------------------------------------------------ reset
    def reset(self, m):
        idx = m.nonzero().squeeze(-1)
        k = len(idx)
        if k == 0:
            return
        d = self.dev
        # how many robots per alliance, and who the other alliance is
        ks = torch.multinomial(self.team_probs.expand(k, -1), 1, generator=self.gen).squeeze(-1) + 1
        self.k[idx] = ks
        kind = torch.multinomial(self.opp_probs.expand(k, -1), 1, generator=self.gen).squeeze(-1)
        self.opp_kind[idx] = kind
        slot = torch.arange(RS, device=d)
        within = (slot % 3)[None] < ks[:, None]
        pres = within & ((self.team == 0)[None] | (kind != OPP_NONE)[:, None])
        self.present[idx] = pres
        # robots: alliance A drives the allowed robots, alliance B any robot
        a_t = self.allowed[torch.randint(0, len(self.allowed), (k, RS), device=d, generator=self.gen)]
        o_t = torch.randint(0, self.NR, (k, RS), device=d, generator=self.gen)
        self.rtype[idx] = torch.where((self.team == 0)[None], a_t, o_t)
        blue = self.rand(k) < 0.5
        sA = torch.where(blue, 1.0, -1.0)
        s = torch.where((self.team == 0)[None], sA[:, None], -sA[:, None])
        self.sgn[idx] = s
        # distinct starting spots per alliance
        perm = torch.argsort(self.rand(k, 2, self.starts.shape[1]), -1)[..., :3]     # [k,2,3]
        st = torch.cat([perm[:, 0], perm[:, 1]], 1)                                 # [k,6]
        p = self.starts[self.rtype[idx], st]                                        # [k,6,2] blue coords
        self.pos[idx] = p * s[..., None]
        self.yaw[idx] = torch.where(s > 0, 0.0, math.pi)
        self.pos[idx] = torch.where(pres[..., None], self.pos[idx], torch.tensor([0.0, 50.0], device=d) + slot[None, :, None].float())
        for name in ['vel', 'omega', 'deploy', 'hopper', 'fly', 'turret', 'tokens', 'feedT', 'holdV', 'intaked', 'shots', 'passes', '_prevIntaked']:
            getattr(self, name)[idx] = 0
        self.hpCool[idx] = 0
        self.holdT[idx] = -99
        for name in ['ready', 'shot', 'feeding']:
            getattr(self, name)[idx] = False
        self.prevActive[idx] = False
        self.cmd[idx] = 0
        self.t[idx] = 0
        self.lastDeact[idx] = -1e9
        self.score[idx] = 0
        self.firstInactive[idx] = 0
        self.bot_skill[idx] = 0.75 + 0.25 * self.rand(k, RS)
        self.bot_def[idx] = self.rand(k, RS) < 0.2
        if self.randomize:
            self.dr_speed[idx] = 0.92 + 0.13 * self.rand(k, RS)
            self.dr_accel[idx] = 0.85 + 0.25 * self.rand(k, RS)
            self.dr_intake[idx] = 0.7 + 0.4 * self.rand(k, RS)
            self.dr_hit[idx] = -0.08 + 0.11 * self.rand(k, RS)
            self.dr_roll[idx] = 0.7 + 0.7 * self.rand(k)
        # FUEL: the staged layout for this many robots, the CHUTES, and 8 preloaded per robot (with
        # no opponent, its preload goes to the NEUTRAL ZONE edges like the game does)
        B, C, Pn = self.B, self.chuteN, self.pre
        for kk in (1, 2, 3):
            g = (ks == kk).nonzero().squeeze(-1)
            if len(g) == 0:
                continue
            ei = idx[g]
            L = self.layouts[kk]
            F = len(L)
            m = len(g)
            bp = torch.zeros(m, B, 2, device=d)
            bp[:, :F] = L
            bst = torch.full((m, B), FIELD, dtype=torch.long, device=d)
            own = torch.zeros(m, B, dtype=torch.long, device=d)
            bst[:, F:F + 2 * C] = CHUTE
            own[:, F + C:F + 2 * C] = 1
            o = F + 2 * C
            for r in [*range(kk), *range(3, 3 + kk)]:
                sl = slice(o, o + Pn)
                o += Pn
                here = pres[g, r]
                bst[:, sl] = torch.where(here[:, None], HELD, FIELD)
                own[:, sl] = torch.where(here[:, None], r, 0)
                edge = torch.stack([(self.rand(m, Pn) - 0.5), (self.rand(m, Pn) - 0.5) * 6.8], -1)
                bp[:, sl] = torch.where(here[:, None, None], bp[:, sl], edge)
            self.bpos[ei], self.bst[ei], self.bown[ei] = bp, bst, own
        self.bvel[idx] = 0
        self.btim[idx] = 0
        self.stored[idx] = torch.where(pres, float(Pn), 0.0)

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
        eye = torch.eye(RS, dtype=torch.bool, device=self.dev)
        hit = (ov > 0).all(-1) & P[:, :, None] & P[:, None, :] & ~eye
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

    def _push_balls_rects(self, p, v, rects, rest):
        """FUEL (circles, [M,1,2]) out of axis-aligned boxes, bouncing."""
        c, h = rects[:, :2], rects[:, 2:]
        rel = p[:, :, None, :] - c
        q = rel.clamp(-h, h)
        dvec = rel - q
        dist = dvec.norm(dim=-1)
        r = self.R
        hit = dist < r
        inside = dist < 1e-6
        pen = h - rel.abs()
        ax = pen[..., 0] < pen[..., 1]
        n_in = torch.where(ax[..., None], torch.stack([rel[..., 0].sign(), torch.zeros_like(rel[..., 0])], -1),
                           torch.stack([torch.zeros_like(rel[..., 1]), rel[..., 1].sign()], -1))
        n_in = torch.where(n_in.abs().sum(-1, keepdim=True) == 0, torch.tensor([1.0, 0.0], device=p.device), n_in)
        depth = torch.where(inside, torch.where(ax, pen[..., 0], pen[..., 1]) + r, r - dist)
        n = torch.where(inside[..., None], n_in, dvec / dist.clamp(min=1e-6)[..., None])
        p = p + torch.where(hit[..., None], n * depth[..., None], 0.0).sum(2)
        vn = (v[:, :, None, :] * n).sum(-1)
        into = hit & (vn < 0)
        v = v + torch.where(into[..., None], -(1 + rest) * vn[..., None] * n, 0.0).sum(2)
        lim = torch.tensor([self.HL - r, self.HW - r], device=p.device)
        over = p.abs() > lim
        v = torch.where(over & (p.sign() == v.sign()), -rest * v, v)
        p = torch.maximum(torch.minimum(p, lim), -lim)
        return p, v

    # ------------------------------------------------------------------ one decision (0.1 s)
    def step(self, act):
        """act: [N,RS,7] network actions (own frame). Returns reward parts per robot [N,RS,5]
        (own alliance pts, their pts, FUEL this robot intaked, own inactive-HUB FUEL, margin after)
        and done [N]."""
        before = torch.stack([self.totals(), self.score[:, :, 2]], -1)
        auto, gap, tele, post, tt = self.phase()
        enabled = (auto | tele)[:, None] & self.present
        a = act.clone()
        a[..., :3] = a[..., :3].clamp(-1, 1)
        self.cmd = torch.where(enabled[..., None], a, torch.zeros_like(a))
        h = self.dt / self.sub
        for _ in range(self.sub):
            self._substep(h, enabled)
            self.t = self.t + h
        self._clock()
        after = torch.stack([self.totals(), self.score[:, :, 2]], -1)
        diff = after - before
        al = self.alliance_idx()
        own = diff.gather(1, al[..., None].expand(-1, -1, 2))
        oth = diff.gather(1, (1 - al)[..., None].expand(-1, -1, 2))
        rew = torch.stack([own[..., 0], oth[..., 0], self.intaked - self._prevIntaked, own[..., 1], self.margin()], -1)
        self._prevIntaked = self.intaked.clone()
        done = self.t >= self.T_DONE - 1e-6
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
    def _substep(self, h, enabled):
        N, d = self.n, self.dev
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
        onf = self.bst == FIELD
        mass = self.spec('mass')
        rate = INTAKE_MAX * self.dr_intake * (self.spec('intakeRate') / 200).clamp(max=1).clamp(min=0.1)
        rate = torch.where(self.spec('intakeRate') < 40, self.spec('intakeRate'), rate)
        row = (self.spec('intakeWidth') / (2 * self.R)).floor().clamp(min=2)  # the rollers grip about a row
        self.tokens = torch.where(running, torch.minimum(self.tokens + rate * h, row), torch.zeros_like(self.tokens))
        room = (cap - self.stored).clamp(min=0)
        take_n = torch.minimum(room, self.tokens.floor())
        # only FUEL near a robot can touch it: a cheap circle test first, then the exact work on those
        reach = torch.hypot(hl + self.deploy * self.spec('intakeReach'), hw) + self.R + 0.02
        ddx = self.bpos[:, None, :, 0] - self.pos[:, :, None, 0]
        ddz = self.bpos[:, None, :, 1] - self.pos[:, :, None, 1]
        cand = (ddx * ddx + ddz * ddz < (reach * reach)[..., None]) & onf[:, None, :] & self.present[..., None]
        e, r, b = cand.nonzero(as_tuple=True)          # sorted by (env, robot, FUEL)
        if len(e):
            dx, dz = ddx[e, r, b], ddz[e, r, b]
            ux, uz, vx_, vz_ = u[e, r, 0], u[e, r, 1], v[e, r, 0], v[e, r, 1]
            lx, lz = dx * ux + dz * uz, dx * vx_ + dz * vz_
            hl_, hw_ = hl[e, r], hw[e, r]
            front = hl_ + self.deploy[e, r] * self.spec('intakeReach')[e, r]
            iw = self.spec('intakeWidth')[e, r] / 2 + 0.02
            near = (lx > -hl_ - self.R) & (lx < front + self.R) & (lz.abs() < hw_ + self.R)
            inI = near & running[e, r] & (lx > hl_ - 0.04) & (lz.abs() < iw)
            # intake: up to take_n per robot, in FUEL order; a FUEL two intakes reach goes to one
            key = e * RS + r
            take = torch.zeros_like(inI)
            si = inI.nonzero(as_tuple=True)[0]
            if len(si):
                ks = key[si]
                rank = torch.arange(len(si), device=self.dev) - torch.searchsorted(ks, ks)
                ok = rank < take_n.flatten()[ks]
                si = si[ok]
                bk = e[si] * self.B + b[si]
                first = torch.full((self.n * self.B,), len(e), dtype=torch.long, device=self.dev).scatter_reduce(0, bk, si, 'amin')
                si = si[first[bk] == si]
                take[si] = True
            tk = take.nonzero(as_tuple=True)[0]
            got = torch.zeros(self.n * RS, device=self.dev).index_add_(0, key[tk], torch.ones(len(tk), device=self.dev)).view(self.n, RS)
            self.stored += got
            self.tokens -= got
            self.intaked += got
            self.bst[e[tk], b[tk]] = HELD
            self.bown[e[tk], b[tk]] = r[tk]
            # FUEL in the mouth that has to wait its turn is held there, not plowed ahead (unless full)
            held = inI & ~take & (room > got)[e, r]
            pi = (near & ~take & ~held).nonzero(as_tuple=True)[0]
            if len(pi):
                e2, r2, b2 = e[pi], r[pi], b[pi]
                lx, lz, hl_, hw_, front = lx[pi], lz[pi], hl_[pi], hw_[pi], front[pi]
                ux, uz, vx_, vz_ = ux[pi], uz[pi], vx_[pi], vz_[pi]
                pen_f = front + self.R - lx
                pen_b = lx + hl_ + self.R
                pen_s = hw_ + self.R - lz.abs()
                m_f = (pen_f <= pen_b) & (pen_f <= pen_s)
                m_b = (pen_b < pen_f) & (pen_b <= pen_s)
                nlx = torch.where(m_f, lx + pen_f, torch.where(m_b, lx - pen_b, lx))
                nlz = torch.where(~m_f & ~m_b, lz + lz.sign() * pen_s, lz)
                px, pz = self.pos[e2, r2, 0], self.pos[e2, r2, 1]
                wx, wz = px + nlx * ux + nlz * vx_, pz + nlx * uz + nlz * vz_
                om = self.omega[e2, r2]
                pv = torch.stack([self.vel[e2, r2, 0] + om * (wz - pz), self.vel[e2, r2, 1] - om * (wx - px)], -1)
                # contact normal (world): out the front, the back or a side of the robot
                sz = lz.sign()
                nrm = torch.stack([torch.where(m_f, ux, torch.where(m_b, -ux, sz * vx_)), torch.where(m_f, uz, torch.where(m_b, -uz, sz * vz_))], -1)
                # bounce off the bumper along the normal (FUEL restitution 0.45); a little slip sideways
                bv = self.bvel[e2, b2]
                rel = pv - bv
                vn = (rel * nrm).sum(-1).clamp(min=0)
                newv = bv + nrm * (vn * 1.45)[:, None] + (rel - nrm * (rel * nrm).sum(-1, keepdim=True)) * 0.2
                self.bpos[e2, b2] = torch.stack([wx, wz], -1)
                self.bvel[e2, b2] = newv
                # the FUEL it shoves takes momentum from the robot
                dp = torch.zeros(self.n * RS, 2, device=self.dev).index_add_(0, e2 * RS + r2, (newv - bv) * 0.215).view(self.n, RS, 2)
                self.vel = self.vel - dp / mass[..., None]

        # ---- shoot / pass: feed while ready
        period = 1.0 / self.spec('bps')
        self.feedT = torch.minimum(self.feedT + h, period * 1.5)
        fire = wantShoot & self.ready & has & (self.feedT >= period)
        self.feeding = wantShoot & self.ready & has
        self.feedT = torch.where(fire, self.feedT - period, self.feedT)
        for r in range(RS):
            f = fire[:, r]
            if not f.any():
                continue
            mine = (self.bst == HELD) & (self.bown == r)
            j = mine.long().argmax(1)
            rows = (f & mine.any(1)).nonzero().squeeze(-1)
            if len(rows) == 0:
                continue
            jj = j[rows]
            self.stored[rows, r] -= 1
            pm = passMode[rows, r]
            self.shots[rows, r] += 1
            self.passes[rows, r] += pm.float()
            spd = self.vel[rows, r].norm(dim=-1)
            p_hit = (0.95 - 0.04 * spd + self.dr_hit[rows, r]).clamp(0.6, 0.97)
            hitb = self.rand(len(rows)) < p_hit
            kind = torch.where(pm, FLY_LAND, torch.where(hitb, FLY_HIT, FLY_MISS))
            dd = dist[rows, r]
            tf = torch.where(pm, 0.5 + 0.1 * dd, 0.35 + 0.12 * dd) * (0.9 + 0.2 * self.rand(len(rows)))
            land = torch.where(pm[:, None], tgt[rows, r] + self.randn(len(rows), 2) * 0.4, hubc[rows, r])
            self.bst[rows, jj] = FLY
            self.bkind[rows, jj] = kind
            self.btim[rows, jj] = tf
            self.bland[rows, jj] = land
            self.bown[rows, jj] = al[rows, r]
            self.bpos[rows, jj] = self.pos[rows, r]
        out = enabled & (cmd[..., 6] > 0.5) & has & (self.deploy > 0.85)
        self._outtake(out, h, u, hl)

        # ---- human players
        act = self.hub_active()
        self.hpCool = (self.hpCool - h).clamp(min=0)
        for a in (0, 1):
            ch = (self.bst == CHUTE) & (self.bown == a)
            go = act[:, a] & (self.hpCool[:, a] <= 0) & ch.any(1)
            rows = go.nonzero().squeeze(-1)
            if len(rows) == 0:
                continue
            jj = ch.long().argmax(1)[rows]
            hitb = self.rand(len(rows)) < 0.5
            hubw = torch.tensor([self.hubX if a == 0 else -self.hubX, 0.0], device=d)
            self.bst[rows, jj] = FLY
            self.bkind[rows, jj] = torch.where(hitb, FLY_HIT, FLY_MISS)
            self.btim[rows, jj] = 1.2
            self.bland[rows, jj] = hubw
            self.bown[rows, jj] = a
            self.hpCool[rows, a] = 1.05

        # ---- FUEL in flight lands / enters the HUB; HUB processing and exits
        fly = self.bst == FLY
        self.btim = torch.where(fly | (self.bst == HUBQ), self.btim - h, self.btim)
        arrive = fly & (self.btim <= 0)
        if arrive.any():
            self._arrive(arrive, act)
        ex = (self.bst == HUBQ) & (self.btim <= 0)
        if ex.any():
            self._exit_hub(ex)

        # ---- crowded FUEL spreads out (FUEL can't stack up in one spot); every other substep is plenty
        self._sub_i = getattr(self, '_sub_i', 0) + 1
        if self._sub_i % 2 == 0:
            self._crowding(h)

        # ---- FUEL rolls: carpet resistance, walls, field elements
        onf = self.bst == FIELD
        sp = self.bvel.norm(dim=-1)
        dec = 0.35 * self.dr_roll[:, None] * h
        k = torch.where(sp <= dec + 0.01, torch.zeros_like(sp), (sp - dec) / sp.clamp(min=1e-6))
        self.bvel = self.bvel * k[..., None]
        moving = onf & (sp > 0.01)
        self.bpos = torch.where(moving[..., None], self.bpos + self.bvel * h, self.bpos)
        mv = moving.nonzero(as_tuple=True)
        if len(mv[0]):
            p, v = self._push_balls_rects(self.bpos[mv][:, None], self.bvel[mv][:, None], self.obs_rects, 0.45)
            self.bpos[mv] = p[:, 0]
            self.bvel[mv] = v[:, 0]

    def _crowding(self, h):
        """FUEL can't stack: FUEL sharing a radius-sized cell overlap (FUEL a diameter apart, like the
        staged rows, never share one); each such pair is pushed apart along the line between them,
        as far as they overlap."""
        onf = self.bst == FIELD
        c = self.R
        gx = ((self.bpos[..., 0] + self.HL) / c).long()
        gz = ((self.bpos[..., 1] + self.HW) / c).long()
        cell = torch.where(onf, gx * 100000 + gz, -1 - torch.arange(self.B, device=self.dev)[None])
        srt, order = cell.sort(1)
        same = srt[:, 1:] == srt[:, :-1]                          # neighbours in sorted order share a cell
        if not same.any():
            return
        ia, ib = order[:, :-1], order[:, 1:]
        pa = self.bpos.gather(1, ia[..., None].expand(-1, -1, 2))
        pb = self.bpos.gather(1, ib[..., None].expand(-1, -1, 2))
        dvec = pa - pb
        dist = dvec.norm(dim=-1)
        rnd = self.randn(*dist.shape, 2)
        nrm = torch.where((dist > 1e-4)[..., None], dvec / dist.clamp(min=1e-4)[..., None], rnd / rnd.norm(dim=-1, keepdim=True).clamp(min=1e-6))
        ov = (2 * self.R - dist).clamp(min=0) * same
        move = nrm * (ov / 2)[..., None]                           # each moves half the overlap
        dp = torch.zeros_like(self.bpos)
        dp.scatter_add_(1, ia[..., None].expand(-1, -1, 2), move)
        dp.scatter_add_(1, ib[..., None].expand(-1, -1, 2), -move)
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

    def _outtake(self, out, h, u, hl):
        for r in range(RS):
            rows = (out[:, r] & (self.rand(self.n) < h / 0.08)).nonzero().squeeze(-1)
            if len(rows) == 0:
                continue
            mine = (self.bst == HELD) & (self.bown == r)
            jj = mine.long().argmax(1)[rows]
            fwd = u[rows, r]
            self.bst[rows, jj] = FIELD
            self.bpos[rows, jj] = self.pos[rows, r] + fwd * (hl[rows, r] + 0.25)[:, None]
            self.bvel[rows, jj] = self.vel[rows, r] + fwd * 2.8
            self.stored[rows, r] -= 1

    def _arrive(self, arrive, act):
        auto, gap, tele, post, tt = self.phase()
        k = self.bkind
        a = self.bown.clamp(max=1)  # alliance index for FUEL in flight (held FUEL stores its robot)
        hit = arrive & (k == FLY_HIT)
        if hit.any():
            actb = act.gather(1, a)
            graceb = (self.t[:, None] - self.lastDeact.gather(1, a)) <= self.grace
            counts = hit & (actb | graceb)
            autoP = auto[:, None] | (gap[:, None] & graceb)
            for alx in (0, 1):
                m = counts & (a == alx)
                self.score[:, alx, 0] += (m & autoP).sum(1).float()
                self.score[:, alx, 1] += (m & ~autoP).sum(1).float()
                self.score[:, alx, 2] += (hit & ~(actb | graceb) & (a == alx)).sum(1).float()
            self.bst = torch.where(hit, HUBQ, self.bst)
            self.btim = torch.where(hit, 0.45 + 0.85 * self.rand(self.n, self.B), self.btim)
        miss = arrive & (k == FLY_MISS)
        if miss.any():
            ang = self.rand(self.n, self.B) * 2 * math.pi
            r = self.hs + 0.15 + 0.8 * self.rand(self.n, self.B)
            dirv = torch.stack([ang.cos(), ang.sin()], -1)
            self.bpos = torch.where(miss[..., None], self.bland + dirv * r[..., None], self.bpos)
            self.bvel = torch.where(miss[..., None], dirv * 0.8, self.bvel)
            self.bst = torch.where(miss, FIELD, self.bst)
        land = arrive & (k == FLY_LAND)
        if land.any():
            self.bpos = torch.where(land[..., None], self.bland, self.bpos)
            self.bvel = torch.where(land[..., None], self.randn(self.n, self.B, 2) * 0.4, self.bvel)
            self.bst = torch.where(land, FIELD, self.bst)

    def _exit_hub(self, ex):
        a = self.bown.clamp(max=1)
        s = torch.where(a == 0, 1.0, -1.0)
        off = (self.rand(self.n, self.B) - 0.5) * (self.exitW - 2 * self.R)
        ang = off / (self.exitW / 2) * 0.5 + (self.rand(self.n, self.B) - 0.5) * 0.8
        sp = self.exitSpeed[0] + (self.exitSpeed[1] - self.exitSpeed[0]) * self.rand(self.n, self.B)
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
        parts.append(Fn.one_hot(self.rtype, self.NR).float())
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
        eye = torch.eye(RS, dtype=torch.bool, device=d)
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
        oh = Fn.one_hot(self.rtype, self.NR).float()[:, None].expand(N, RS, RS, self.NR)
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
        onf = (self.bst == FIELD)[:, None, :].expand(N, RS, self.B)
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
    def bot_actions(self, envs=None):
        """Simple scripted robots for every slot: [N,RS,7] actions (own frame). Most score
        (collect the nearest FUEL, score in their zone while their HUB is active); a few defend
        (get between the other ALLIANCE's most loaded robot and its HUB). envs: only the matches
        that have scripted robots (bool [N]); the others get zeros."""
        N, d = self.n, self.dev
        if envs is None:
            envs = torch.ones(N, dtype=torch.bool, device=d)
        E = envs.nonzero().squeeze(-1)
        s = self.sgn
        x, z = s * self.pos[..., 0], s * self.pos[..., 1]
        yaw = self.yaw + torch.where(s > 0, 0.0, math.pi)
        al = self.alliance_idx()
        act = self.hub_active().gather(1, al)
        nc = self.next_change().gather(1, al) * self.T_SHIFT
        cap = self.spec('capMax')
        n_ = self.stored
        auto, gap, tele, post, tt = self.phase()
        go_score = (n_ >= 0.6 * cap) | ((n_ >= 4) & (act | (nc < 4))) | (auto[:, None] & (n_ > 0) & (self.t[:, None] > 12))
        # the FUEL to go for (only in the matches with scripted robots: it's the costly part)
        fx, fz = torch.zeros_like(x), torch.zeros_like(z)
        if len(E):
            onf = (self.bst[E] == FIELD)
            wb = self.bpos[E]                                            # [n,B,2] world
            # skip FUEL tucked against a wall or a field element (no path planning here); the field is
            # point-symmetric, so this doesn't depend on whose frame it's seen in
            Rr = self.obs_rects
            gapb = torch.maximum((wb[..., None, 0] - Rr[:, 0]).abs() - Rr[:, 2], (wb[..., None, 1] - Rr[:, 1]).abs() - Rr[:, 3]).min(-1).values
            ok = onf & (gapb > 0.35) & (wb[..., 0].abs() < self.HL - 0.4) & (wb[..., 1].abs() < self.HW - 0.4)
            sE = s[E]
            bx = sE[..., None] * wb[:, None, :, 0]
            bz = sE[..., None] * wb[:, None, :, 1]
            okr = ok[:, None, :].expand_as(bx)
            d2 = torch.where(okr, (bx - x[E][..., None]) ** 2 + (bz - z[E][..., None]) ** 2, torch.full_like(bx, 1e9))
            # prefer FUEL with company (dense patches), not a lone ball it'll knock away
            GX, GZ = 16, 8
            gi = (((bx + self.HL) / (2 * self.HL) * GX).long().clamp(0, GX - 1) * GZ + ((bz + self.HW) / (2 * self.HW) * GZ).long().clamp(0, GZ - 1))
            dens = torch.zeros(len(E), RS, GX * GZ, device=d).scatter_add_(-1, gi, okr.float())
            jj = (d2 - 0.6 * dens.gather(-1, gi).clamp(max=12)).argmin(-1)
            fx[E] = bx.gather(-1, jj[..., None]).squeeze(-1)
            fz[E] = bz.gather(-1, jj[..., None]).squeeze(-1)
        tx = torch.where(go_score, torch.full_like(x, self.lineX - 1.2), fx)
        tz = torch.where(go_score, torch.where(z > 0, 1.7, -1.7), fz)
        # defenders: between the most loaded opponent and its HUB (own frame: their HUB is at -hubX)
        foe_load = torch.where((al[:, :, None] != al[:, None, :]) & self.present[:, None, :], self.stored[:, None, :].expand(N, RS, RS), torch.full((N, RS, RS), -1.0, device=d))
        Rr = self.obs_rects
        tgt = foe_load.argmax(-1)
        ox = s * self.pos[..., 0].gather(1, tgt)
        oz = s * self.pos[..., 1].gather(1, tgt)
        dfx, dfz = ox + (-self.hubX - ox) * 0.35, oz * 0.65
        defend = self.bot_def & tele[:, None]
        tx, tz = torch.where(defend, dfx, tx), torch.where(defend, dfz, tz)
        dx, dz = tx - x, tz - z
        dist = torch.hypot(dx, dz).clamp(min=1e-3)
        vx, vz = dx / dist, dz / dist
        rx = (x[..., None] - Rr[:, 0]).abs() - Rr[:, 2]
        rz = (z[..., None] - Rr[:, 1]).abs() - Rr[:, 3]
        g = torch.maximum(rx, rz)
        close = g < 0.9
        px = torch.where(close, (x[..., None] - Rr[:, 0]).sign() * (rx > rz).float() / g.clamp(min=0.05), 0.0).sum(-1)
        pz = torch.where(close, (z[..., None] - Rr[:, 1]).sign() * (rz >= rx).float() / g.clamp(min=0.05), 0.0).sum(-1)
        vx, vz = vx + 0.06 * px, vz + 0.06 * pz
        # stuck (asking to move, not moving): sidestep
        stuck = (self.vel.norm(dim=-1) < 0.15) & (dist > 0.6) & self.present
        side = torch.where(self.bot_skill > 0.875, 1.0, -1.0)
        vx, vz = torch.where(stuck, vx - side * vz * 1.5, vx), torch.where(stuck, vz + side * vx * 1.5, vz)
        nv = torch.hypot(vx, vz).clamp(min=1e-3)
        sp = torch.where(go_score | defend, (dist / 0.8).clamp(0.0, 1.0), (dist / 1.5).clamp(0.4, 1.0)) * self.bot_skill
        out = torch.zeros(N, RS, 7, device=d)
        out[..., 0], out[..., 1] = vx / nv * sp, vz / nv * sp
        want = torch.atan2(-dz, dx)
        out[..., 2] = (wrap(want - yaw) * 2).clamp(-1, 1) * (~go_score).float()
        collecting = ~go_score & ~defend
        out[..., 3] = collecting.float()
        inZ = self.in_zone(self.sgn)
        out[..., 4] = (go_score & ~defend & inZ & (act | (nc < 1.0))).float()
        return out * envs[:, None, None].float()

    # ------------------------------------------------------------------ live view
    def frame(self, envs):
        """A bird's-eye snapshot of some matches for the dashboard (plain Python data)."""
        e = torch.as_tensor(envs, device=self.dev)
        pos = self.pos[e].cpu()
        yaw = self.yaw[e].cpu()
        pres = self.present[e].cpu()
        typ = self.rtype[e].cpu()
        sg = self.sgn[e].cpu()
        sto = self.stored[e].cpu()
        learn = self.learning()[e].cpu()
        bst = (self.bst[e] == FIELD).cpu()
        bp = (self.bpos[e] * 100).round().clamp(-32000, 32000).short().cpu()
        tot = self.totals()[e].cpu()
        act = self.hub_active()[e].cpu()
        t = self.t[e].cpu()
        kinds = self.opp_kind[e].cpu()
        out = []
        for i in range(len(envs)):
            balls = bp[i][bst[i]].numpy().astype('<i2').tobytes()
            out.append({
                't': round(float(t[i]), 2), 'score': [int(tot[i, 0]), int(tot[i, 1])], 'active': [bool(act[i, 0]), bool(act[i, 1])],
                'opp': int(kinds[i]),
                'robots': [[round(float(pos[i, r, 0]), 3), round(float(pos[i, r, 1]), 3), round(float(yaw[i, r]), 3),
                            0 if sg[i, r] > 0 else 1, self.order[int(typ[i, r])], int(sto[i, r]), int(learn[i, r])]
                           for r in range(RS) if pres[i, r]],
                'balls': balls,
            })
        return out
