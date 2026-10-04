"""Batched REBUILT simulator on the GPU (PyTorch), for training the neural-net driver fast.

Thousands of 1v1 matches run at once as tensor math. It is a simplified version of the real game:
  * robots: swerve drive with the real speed / acceleration / turn limits, round-ish bodies that
    collide with the walls, HUBS, TRENCH posts, TOWERS (and TRENCH arms for tall robots) and each
    other; intake, hopper capacity, flywheel spin-up, turret / chassis aiming, shot rate
  * FUEL: points that roll and slow down on the carpet, bounce off walls and field elements and
    get pushed by robots (no FUEL-FUEL contact); shots, passes, misses and the HUB exits
  * match: AUTO, the 3 s gap, TELEOP shifts with the active-HUB rules, 3 s scoring grace, human
    players throwing from the CHUTE
What the network sees is built exactly like js/nn/obs.js, and it gives the same 7 controls, so a
network trained here drives the real game (and is fine-tuned there with tools/nn/train.py).

Robot specs, the field and the FUEL layout come from tools/nn/gpusim-params.json, written by
tools/nn/gpusim-export.mjs from the real game.
"""
import json
import math
from pathlib import Path

import torch

PARAMS = Path(__file__).with_name('gpusim-params.json')

FIELD, HELD, FLY, HUBQ, CHUTE = 0, 1, 2, 3, 4
FLY_HIT, FLY_MISS, FLY_LAND = 0, 1, 2
OPP_NONE, OPP_BOT, OPP_SNAP, OPP_SELF = 0, 1, 2, 3
SUB = 4  # physics substeps per decision


def wrap(a):
    return torch.remainder(a + math.pi, 2 * math.pi) - math.pi


def approach(cur, tgt, step):
    return cur + torch.clamp(tgt - cur, -step, step)


class GpuSim:
    def __init__(self, n, device='cuda', robots=None, seed=0, params=PARAMS, substeps=SUB):
        P = json.loads(Path(params).read_text())
        self.P = P
        self.n = n
        self.sub = substeps  # physics substeps per 0.1 s decision (fewer = faster, coarser)
        self.dev = torch.device(device)
        self.gen = torch.Generator(device=self.dev)
        self.gen.manual_seed(seed)
        d = self.dev
        f = P['field']
        self.HL, self.HW = f['halfL'], f['halfW']
        self.lineX = f['allianceLineX']          # blue alliance line (negative x)
        self.hubX = f['hubX']                    # blue HUB center x (negative)
        self.hs = P['hub']['size'] / 2
        self.R = P['fuel']['radius']
        self.dt = P['obs']['decisionDt']
        self.D = P['obs']['dim']
        self.layout = P['obs']['layout']
        T = P['timing']
        self.T_AUTO, self.T_GAP, self.T_TELE, self.T_POST = T['auto'], T['autoGap'], T['teleop'], T['post']
        self.T_TRANS, self.T_SHIFT, self.T_END = T['transition'], T['shift'], T['endgame']
        self.TOTAL = self.T_AUTO + self.T_TELE
        self.grace = P['hub']['scoreGrace']

        # ---- robot specs, indexed by robot type
        self.order = P['robotOrder']
        self.NR = len(self.order)
        spec = lambda k: torch.tensor([float(P['robots'][r][k]) for r in self.order], device=d)
        self.S = {k: spec(k) for k in ['halfL', 'halfW', 'maxSpeed', 'maxAccel', 'maxOmega', 'maxAlpha', 'intakeWidth', 'intakeReach',
                                       'intakeRate', 'deployTime', 'capIn', 'capOut', 'capMax', 'bps', 'spinTau', 'speedMax',
                                       'turretRate', 'turretRange', 'aimOffset', 'turrets']}
        self.S['turret'] = spec('turret') > 0.5
        self.S['latched'] = spec('latched') > 0.5
        self.S['fits'] = spec('fitsTrench') > 0.5
        self.S['rr'] = torch.minimum(self.S['halfL'], self.S['halfW']) * 0.55 + torch.hypot(self.S['halfL'], self.S['halfW']) * 0.45
        # flywheel speed needed vs distance (0.5 m steps); out of range = nan
        tab = lambda k: torch.tensor([[float('nan') if v is None else v for v in P['robots'][r][k]] for r in self.order], device=d)
        self.hubV, self.passV = tab('hubV'), tab('passV')
        self.starts = torch.tensor([[P['robots'][r]['starts'][k] for k in P['startOrder']] for r in self.order], device=d)  # [NR,5,2] blue
        self.allowed = torch.tensor([self.order.index(r) for r in (robots or self.order)], device=d)

        # ---- field geometry
        self.obs_rects = torch.tensor([[o['x'], o['z'], o['hx'], o['hz']] for o in P['obstacles']], device=d)  # [K,4]
        self.arms = torch.tensor([[a['x'], a['z'], a['hx'], a['hz']] for a in P['trenchArms']], device=d)
        self.layoutF = torch.tensor(P['fuel']['field'], device=d)  # [F,2]
        self.F = len(self.layoutF)
        self.chuteN = P['fuel']['perChute']
        self.pre = P['fuel']['preload']
        self.B = self.F + 2 * self.chuteN + 2 * self.pre
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
        return self.S[k][self.rtype]  # [N,2]

    def _alloc(self):
        n, d, B = self.n, self.dev, self.B
        z = lambda *s: torch.zeros(*s, device=d)
        self.rtype = torch.zeros(n, 2, dtype=torch.long, device=d)
        self.sgn = z(n, 2)               # +1 blue, -1 red
        self.present = torch.ones(n, 2, dtype=torch.bool, device=d)
        self.pos, self.vel = z(n, 2, 2), z(n, 2, 2)
        self.yaw, self.omega = z(n, 2), z(n, 2)
        self.deploy, self.hopper, self.fly, self.turret = z(n, 2), z(n, 2), z(n, 2), z(n, 2)
        self.stored, self.tokens, self.feedT = z(n, 2), z(n, 2), z(n, 2)
        self.ready = torch.zeros(n, 2, dtype=torch.bool, device=d)
        self.shot = torch.zeros(n, 2, dtype=torch.bool, device=d)
        self.feeding = torch.zeros(n, 2, dtype=torch.bool, device=d)
        self.holdV, self.holdT = z(n, 2), z(n, 2) - 99
        self.cmd = z(n, 2, 7)
        self.intaked, self.shots, self.passes = z(n, 2), z(n, 2), z(n, 2)
        self._prevIntaked = z(n, 2)
        self.t = z(n)                      # seconds since AUTO started
        self.firstInactive = torch.zeros(n, dtype=torch.long, device=d)  # alliance index 0 blue, 1 red
        self.prevActive = torch.zeros(n, 2, dtype=torch.bool, device=d)
        self.lastDeact = z(n, 2) - 1e9
        self.score = z(n, 2, 3)            # per alliance: autoFuel, teleFuel, inactiveFuel
        self.hpCool = z(n, 2)
        self.opp_kind = torch.zeros(n, dtype=torch.long, device=d)
        self.bot_skill = z(n) + 1
        self.bpos, self.bvel = z(n, B, 2), z(n, B, 2)
        self.bst = torch.zeros(n, B, dtype=torch.long, device=d)
        self.bown = torch.zeros(n, B, dtype=torch.long, device=d)  # robot index (HELD) / alliance index (FLY, HUBQ, CHUTE)
        self.btim = z(n, B)
        self.bland = z(n, B, 2)
        self.bkind = torch.zeros(n, B, dtype=torch.long, device=d)

    def alliance_idx(self):
        return (self.sgn < 0).long()  # [N,2] 0 blue 1 red

    # ------------------------------------------------------------------ reset
    def reset(self, m):
        """Start new matches in the envs where m (bool [N]) is set."""
        idx = m.nonzero().squeeze(-1)
        k = len(idx)
        if k == 0:
            return
        d = self.dev
        # robots: the agent drives one of the allowed robots, the opponent any robot
        a_t = self.allowed[torch.randint(0, len(self.allowed), (k,), device=d, generator=self.gen)]
        o_t = torch.randint(0, self.NR, (k,), device=d, generator=self.gen)
        self.rtype[idx] = torch.stack([a_t, o_t], 1)
        blue = self.rand(k) < 0.5
        s0 = torch.where(blue, 1.0, -1.0)
        self.sgn[idx] = torch.stack([s0, -s0], 1)
        kind = torch.multinomial(self.opp_probs.expand(k, -1), 1, generator=self.gen).squeeze(-1)
        self.opp_kind[idx] = kind
        self.present[idx, 1] = kind != OPP_NONE
        self.bot_skill[idx] = 0.75 + 0.25 * self.rand(k)
        for r in range(2):
            st = torch.randint(0, self.starts.shape[1], (k,), device=d, generator=self.gen)
            p = self.starts[self.rtype[idx, r], st]  # blue coords
            s = self.sgn[idx, r]
            self.pos[idx, r] = p * s[:, None]
            self.yaw[idx, r] = torch.where(s > 0, 0.0, math.pi)
        far = ~self.present[idx, 1]
        self.pos[idx[far], 1] = torch.tensor([0.0, 50.0], device=d)  # parked off the field
        for name in ['vel', 'omega', 'deploy', 'hopper', 'fly', 'turret', 'tokens', 'feedT', 'holdV', 'intaked', 'shots', 'passes', 'hpCool', '_prevIntaked']:
            getattr(self, name)[idx] = 0
        self.holdT[idx] = -99
        for name in ['ready', 'shot', 'feeding', 'prevActive']:
            getattr(self, name)[idx] = False
        self.cmd[idx] = 0
        self.t[idx] = 0
        self.lastDeact[idx] = -1e9
        self.score[idx] = 0
        self.firstInactive[idx] = 0
        # FUEL: the staged layout, the CHUTES, and 8 preloaded in each robot
        B, F, C, Pn = self.B, self.F, self.chuteN, self.pre
        bp = torch.zeros(k, B, 2, device=d)
        bp[:, :F] = self.layoutF + self.randn(k, F, 2) * 0.004
        st = torch.full((k, B), FIELD, dtype=torch.long, device=d)
        own = torch.zeros(k, B, dtype=torch.long, device=d)
        st[:, F:F + 2 * C] = CHUTE
        own[:, F:F + C] = 0
        own[:, F + C:F + 2 * C] = 1
        st[:, F + 2 * C:] = HELD
        own[:, F + 2 * C:F + 2 * C + Pn] = 0
        own[:, F + 2 * C + Pn:] = 1
        # no opponent robot: its preload goes to the NEUTRAL ZONE like the game does
        if far.any():
            j = far.nonzero().squeeze(-1)
            sl = slice(F + 2 * C + Pn, B)
            st[j, sl] = FIELD
            bp[j, sl] = torch.stack([(self.rand(len(j), Pn) - 0.5), (self.rand(len(j), Pn) - 0.5) * 6.8], -1)
        self.bpos[idx], self.bst[idx], self.bown[idx] = bp, st, own
        self.bvel[idx] = 0
        self.btim[idx] = 0
        self.stored[idx] = torch.stack([torch.full((k,), float(Pn), device=d), torch.where(far, 0.0, float(Pn))], 1)

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
        """HUB active for alliance index a ([N] or scalar) at shift index si ([N])."""
        odd = si % 2 == 1
        fi = self.firstInactive
        teleAct = torch.where((si == 0) | (si == 5), True, torch.where(odd, a != fi, a == fi))
        return teleAct

    def hub_active(self):
        """[N,2] by alliance index."""
        auto, gap, tele, post, tt = self.phase()
        si = self.shift_index(tele, tt)
        out = []
        for a in (0, 1):
            out.append(torch.where(auto, True, torch.where(tele, self._active_at(si, a), False)))
        return torch.stack(out, 1)

    def next_change(self):
        """seconds until each alliance's HUB status changes, normalized like obs.js (1 = never)."""
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

    # ------------------------------------------------------------------ geometry
    def in_zone(self, alliance_sign):
        """[N,2]: any BUMPER corner of each robot in the zone of the alliance with this sign."""
        x_own = self.pos[..., 0] * alliance_sign
        ext = self.spec('halfL') * self.yaw.cos().abs() + self.spec('halfW') * self.yaw.sin().abs()
        return x_own - ext <= self.lineX

    def _push_rects(self, p, v, rad, rects, rest=0.0):
        """Push circles (p [N,M,2], radius rad [N,M] or float) out of axis-aligned rects [K,4]."""
        if rects.numel() == 0:
            return p, v
        c, h = rects[:, :2], rects[:, 2:]
        rel = p[:, :, None, :] - c                       # [N,M,K,2]
        q = rel.clamp(-h, h)                             # closest point (relative)
        dvec = rel - q
        dist = dvec.norm(dim=-1)
        r = rad if isinstance(rad, float) else rad[:, :, None]
        hit = dist < r
        inside = dist < 1e-6
        # inside the rect: push out the short way
        pen = h - rel.abs()                              # [N,M,K,2]
        ax = (pen[..., 0] < pen[..., 1])
        n_in = torch.where(ax[..., None], torch.stack([rel[..., 0].sign(), torch.zeros_like(rel[..., 0])], -1),
                           torch.stack([torch.zeros_like(rel[..., 1]), rel[..., 1].sign()], -1))
        n_in = torch.where(n_in.abs().sum(-1, keepdim=True) == 0, torch.tensor([1.0, 0.0], device=p.device), n_in)
        depth_in = torch.where(ax, pen[..., 0], pen[..., 1]) + r
        n_out = dvec / dist.clamp(min=1e-6)[..., None]
        depth_out = r - dist
        n = torch.where(inside[..., None], n_in, n_out)
        depth = torch.where(inside, depth_in, depth_out)
        push = torch.where(hit[..., None], n * depth[..., None], 0.0).sum(2)
        p = p + push
        # remove (or bounce) the velocity into the surface
        vn = (v[:, :, None, :] * n).sum(-1)              # [N,M,K]
        into = hit & (vn < 0)
        dv = torch.where(into[..., None], -(1 + rest) * vn[..., None] * n, 0.0).sum(2)
        return p, v + dv

    def _walls(self, p, v, rad, rest=0.0):
        lim = torch.stack([self.HL - rad, self.HW - rad], -1) if not isinstance(rad, float) else torch.tensor([self.HL - rad, self.HW - rad], device=p.device)
        over = p.abs() > lim
        v = torch.where(over & (p.sign() == v.sign()), -rest * v, v)
        p = torch.maximum(torch.minimum(p, lim), -lim)
        return p, v

    # ------------------------------------------------------------------ one decision (0.1 s)
    def step(self, act):
        """act: [N,2,7] network actions (own frame) for both robots. Returns reward parts per
        robot [N,2,4] (own pts, their pts, intaked, inactive-HUB FUEL) and done [N]."""
        before = self._snap()
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
        after = self._snap()
        diff = after - before                                            # [N, alliance, 2]
        al = self.alliance_idx()                                         # [N, robot]
        own = diff.gather(1, al[..., None].expand(-1, -1, 2))            # [N, robot, 2]
        oth = diff.gather(1, (1 - al)[..., None].expand(-1, -1, 2))
        rew = torch.stack([own[..., 0], oth[..., 0], self.intaked - self._prevIntaked, own[..., 1]], -1)
        self._prevIntaked = self.intaked.clone()
        done = self.t >= self.T_AUTO + self.T_GAP + self.T_TELE + self.T_POST - 1e-6
        return rew, done

    def _snap(self):
        return torch.stack([self.totals(), self.score[:, :, 2]], -1)  # [N, alliance, 2]

    def _clock(self):
        auto, gap, tele, post, tt = self.phase()
        act = self.hub_active()
        fell = self.prevActive & ~act
        self.lastDeact = torch.where(fell, self.t[:, None], self.lastDeact)
        self.prevActive = act
        # decide the first inactive HUB when TELEOP starts (more AUTO FUEL -> inactive first)
        start = tele & (tt < self.dt + 1e-6)
        b, r = self.score[:, 0, 0], self.score[:, 1, 0]
        fi = torch.where(b > r, 0, torch.where(r > b, 1, (self.rand(self.n) < 0.5).long()))
        self.firstInactive = torch.where(start, fi, self.firstInactive)

    # ------------------------------------------------------------------ physics substep
    def _substep(self, h, enabled):
        N, d = self.n, self.dev
        cmd = self.cmd
        s = self.sgn
        ms, ma, mo, mal = self.spec('maxSpeed'), self.spec('maxAccel'), self.spec('maxOmega'), self.spec('maxAlpha')
        hl, hw = self.spec('halfL'), self.spec('halfW')
        has = self.stored > 0.5
        wantShoot = enabled & ((cmd[..., 4] > 0.5) | (cmd[..., 5] > 0.5))
        wantPass = cmd[..., 5] > 0.5
        al = self.alliance_idx()
        inZ = self.in_zone(s)
        passMode = wantPass | ~inZ

        # ---- shot target and needed speed
        hubc = torch.stack([s * self.hubX, torch.zeros_like(s)], -1)              # own HUB center (world)
        cx = s * (-self.HL + 1.7)
        cz = torch.where((self.pos[..., 1] - (self.HW - 1.8)).abs() < (self.pos[..., 1] + (self.HW - 1.8)).abs(), self.HW - 1.8, -(self.HW - 1.8))
        passc = torch.stack([cx, cz], -1)
        tgt = torch.where(passMode[..., None], passc, hubc)
        rel = tgt - self.pos
        dist = rel.norm(dim=-1)
        psi = torch.atan2(-rel[..., 1], rel[..., 0])                              # world yaw to target
        need = self._need_speed(dist, passMode)                                   # nan if out of range
        prespin = enabled & has & inZ
        aiming = (wantShoot | prespin) & has & ~torch.isnan(need)
        self.shot = aiming
        turret = self.spec('turret')
        fixed = ~turret
        # chassis heading controller for chassis-aimed shooters while shooting
        aimDir = self.yaw + self.spec('aimOffset')
        err = wrap(psi - aimDir)
        w_aim = torch.minimum(mo, torch.minimum(torch.sqrt(2 * 0.7 * mal * err.abs()), 7 * err.abs())) * err.sign()
        omega_cmd = cmd[..., 2] * mo
        omega_cmd = torch.where(fixed & wantShoot & aiming, w_aim, omega_cmd)
        # turret
        relang = wrap(psi - self.yaw)
        lim = self.spec('turretRange') * math.pi / 180
        twin = self.spec('turrets') > 1.5
        goal = torch.where(twin, relang, relang.clamp(-lim, lim))
        self.turret = torch.where(turret & aiming, approach(self.turret, goal, self.spec('turretRate') * math.pi / 180 * h), self.turret)
        aimErr = torch.where(turret, wrap(psi - self.yaw - self.turret), err)
        tol = torch.where(passMode, 4 * math.pi / 180, torch.maximum(torch.full_like(dist, 0.6 * math.pi / 180), torch.atan2(torch.full_like(dist, 0.14), dist)))
        # flywheel
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
        vx = cmd[..., 0] * s * ms
        vz = cmd[..., 1] * s * ms
        sp = torch.hypot(vx, vz)
        k = torch.where(sp > ms, ms / sp.clamp(min=1e-6), torch.ones_like(sp))
        vx, vz = vx * k, vz * k
        w = omega_cmd.clamp(-mo, mo)
        rd = torch.hypot(hl, hw)
        wheel = torch.hypot(vx, vz) + w.abs() * rd
        k = torch.where(wheel > ms, ms / wheel.clamp(min=1e-6), torch.ones_like(wheel))
        vx, vz, w = vx * k, vz * k, w * k
        en = enabled.float()
        vx, vz, w = vx * en, vz * en, w * en
        dvx, dvz = vx - self.vel[..., 0], vz - self.vel[..., 1]
        dm = torch.hypot(dvx, dvz)
        k = torch.where(dm > ma * h, ma * h / dm.clamp(min=1e-6), torch.ones_like(dm))
        self.vel = self.vel + torch.stack([dvx * k, dvz * k], -1)
        self.omega = approach(self.omega, w, mal * h)
        self.pos = self.pos + self.vel * h
        self.yaw = wrap(self.yaw + self.omega * h)
        # collisions: walls, field elements, TRENCH arms for tall robots, the other robot
        rr = self.spec('rr')
        self.pos, self.vel = self._push_rects(self.pos, self.vel, rr, self.obs_rects)
        tall = ~self.spec('fits')
        if tall.any():
            p2, v2 = self._push_rects(self.pos, self.vel, rr, self.arms)
            self.pos = torch.where(tall[..., None], p2, self.pos)
            self.vel = torch.where(tall[..., None], v2, self.vel)
        self.pos, self.vel = self._walls(self.pos, self.vel, rr)
        both = self.present.all(-1)
        dpos = self.pos[:, 0] - self.pos[:, 1]
        dd = dpos.norm(dim=-1)
        minD = rr[:, 0] + rr[:, 1]
        hit = both & (dd < minD)
        nrm = dpos / dd.clamp(min=1e-6)[:, None]
        corr = torch.where(hit, (minD - dd) / 2, 0.0)[:, None] * nrm
        self.pos = self.pos + torch.stack([corr, -corr], 1)
        rv = ((self.vel[:, 0] - self.vel[:, 1]) * nrm).sum(-1)
        imp = torch.where(hit & (rv < 0), rv / 2, 0.0)[:, None] * nrm
        self.vel = self.vel - torch.stack([imp, -imp], 1)

        # ---- intake and hopper
        want = enabled & ((cmd[..., 3] > 0.5) | (cmd[..., 6] > 0.5))
        want = want | (self.spec('latched') & (self.deploy >= 1))
        self.deploy = approach(self.deploy, want.float(), h / self.spec('deployTime'))
        capIn, capOut = self.spec('capIn'), self.spec('capOut')
        hopWant = torch.where(self.spec('latched'), torch.where((self.hopper > 0.02) | (self.deploy > 0.3), 1.0, 0.0), self.deploy)
        need_h = ((self.stored - capIn) / (capOut - capIn).clamp(min=1)).clamp(0, 1)
        hopWant = torch.maximum(hopWant, need_h)
        self.hopper = approach(self.hopper, hopWant, h / 0.45)
        cap = torch.floor(capIn + (capOut - capIn) * self.hopper + 1e-6)
        running = enabled & (cmd[..., 3] > 0.5) & (self.deploy > 0.85)

        # ---- FUEL on the field vs robots: intake, or get pushed
        onf = self.bst == FIELD
        c, sn = self.yaw.cos(), self.yaw.sin()
        for r in range(2):
            pr = self.present[:, r]
            dx = self.bpos[..., 0] - self.pos[:, r, None, 0]
            dz = self.bpos[..., 1] - self.pos[:, r, None, 1]
            lx = dx * c[:, r, None] - dz * sn[:, r, None]
            lz = dx * sn[:, r, None] + dz * c[:, r, None]
            front = hl[:, r] + self.deploy[:, r] * self.spec('intakeReach')[:, r]
            hwr = hw[:, r]
            iw = self.spec('intakeWidth')[:, r] / 2 + 0.02
            near = onf & pr[:, None] & (lx > -hl[:, r, None] - self.R) & (lx < front[:, None] + self.R) & (lz.abs() < hwr[:, None] + self.R)
            # intake: in front, across the intake width
            inIntake = near & running[:, r, None] & (lx > hl[:, r, None] - 0.04) & (lz.abs() < iw[:, None])
            rate = self.spec('intakeRate')[:, r].clamp(max=40)
            self.tokens[:, r] = torch.where(running[:, r], torch.minimum(self.tokens[:, r] + rate * h, torch.full_like(rate, 4.0)), torch.zeros_like(rate))
            room = (cap[:, r] - self.stored[:, r]).clamp(min=0)
            take_n = torch.minimum(room, self.tokens[:, r].floor())
            take = inIntake & (torch.cumsum(inIntake.long(), 1) <= take_n[:, None])
            got = take.sum(1).float()
            self.stored[:, r] += got
            self.tokens[:, r] -= got
            self.intaked[:, r] += got
            self.bst = torch.where(take, HELD, self.bst)
            self.bown = torch.where(take, r, self.bown)
            # pushed: out of the robot's footprint (+ deployed intake) the short way
            push = near & ~take
            if push.any():
                pen_f = front[:, None] + self.R - lx
                pen_b = lx + hl[:, r, None] + self.R
                pen_s = hwr[:, None] + self.R - lz.abs()
                m_f = (pen_f <= pen_b) & (pen_f <= pen_s)
                m_b = (pen_b < pen_f) & (pen_b <= pen_s)
                nlx = torch.where(m_f, lx + pen_f, torch.where(m_b, lx - pen_b, lx))
                nlz = torch.where(~m_f & ~m_b, lz + lz.sign() * pen_s, lz)
                # back to world
                wx = self.pos[:, r, None, 0] + nlx * c[:, r, None] + nlz * sn[:, r, None]
                wz = self.pos[:, r, None, 1] - nlx * sn[:, r, None] + nlz * c[:, r, None]
                self.bpos = torch.where(push[..., None], torch.stack([wx, wz], -1), self.bpos)
                # take on the robot's velocity at that point (a bit faster: it's a kick)
                pvx = self.vel[:, r, None, 0] + self.omega[:, r, None] * (wz - self.pos[:, r, None, 1])
                pvz = self.vel[:, r, None, 1] - self.omega[:, r, None] * (wx - self.pos[:, r, None, 0])
                kick = torch.stack([pvx, pvz], -1) * 1.15
                self.bvel = torch.where(push[..., None], kick + 0.2 * self.bvel, self.bvel)
            onf = self.bst == FIELD

        # ---- shoot / pass: feed while ready
        period = 1.0 / self.spec('bps')
        self.feedT = torch.minimum(self.feedT + h, period * 1.5)
        fire = wantShoot & self.ready & has & (self.feedT >= period)
        self.feeding = wantShoot & self.ready & has
        self.feedT = torch.where(fire, self.feedT - period, self.feedT)
        for r in range(2):
            f = fire[:, r]
            if not f.any():
                continue
            mine = (self.bst == HELD) & (self.bown == r)
            j = mine.long().argmax(1)
            ok = f & mine.any(1)
            rows = ok.nonzero().squeeze(-1)
            if len(rows) == 0:
                continue
            jj = j[rows]
            self.stored[rows, r] -= 1
            pm = passMode[rows, r]
            self.shots[rows, r] += 1
            self.passes[rows, r] += pm.float()
            spd = self.vel[rows, r].norm(dim=-1)
            p_hit = (0.95 - 0.04 * spd).clamp(0.75, 0.95)
            hitb = self.rand(len(rows)) < p_hit
            kind = torch.where(pm, FLY_LAND, torch.where(hitb, FLY_HIT, FLY_MISS))
            dd = dist[rows, r]
            tf = torch.where(pm, 0.5 + 0.1 * dd, 0.35 + 0.12 * dd)
            land = torch.where(pm[:, None], tgt[rows, r] + self.randn(len(rows), 2) * 0.4, hubc[rows, r])
            self.bst[rows, jj] = FLY
            self.bkind[rows, jj] = kind
            self.btim[rows, jj] = tf
            self.bland[rows, jj] = land
            self.bown[rows, jj] = al[rows, r]
            self.bpos[rows, jj] = self.pos[rows, r]
        # outtake: spit FUEL out the front
        out = enabled & (cmd[..., 6] > 0.5) & has & (self.deploy > 0.85)
        self._outtake(out, h, c, sn, hl)

        # ---- human players throw from the CHUTE while their HUB is active
        act = self.hub_active()
        self.hpCool = (self.hpCool - h).clamp(min=0)
        for a in (0, 1):
            go = act[:, a] & (self.hpCool[:, a] <= 0)
            ch = (self.bst == CHUTE) & (self.bown == a)
            go = go & ch.any(1)
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

        # ---- FUEL rolls: carpet resistance, walls, field elements
        onf = self.bst == FIELD
        sp = self.bvel.norm(dim=-1)
        dec = 0.35 * h
        k = torch.where(sp <= dec + 0.01, torch.zeros_like(sp), (sp - dec) / sp.clamp(min=1e-6))
        self.bvel = self.bvel * k[..., None]
        moving = onf & (sp > 0.01)
        self.bpos = torch.where(moving[..., None], self.bpos + self.bvel * h, self.bpos)
        # only rolling FUEL can hit anything (most of it sits still), so collide just those
        mv = moving.nonzero(as_tuple=True)
        if len(mv[0]):
            p, v = self.bpos[mv][:, None], self.bvel[mv][:, None]
            p, v = self._push_rects(p, v, self.R, self.obs_rects, rest=0.45)
            p, v = self._walls(p, v, self.R, rest=0.45)
            self.bpos[mv] = p[:, 0]
            self.bvel[mv] = v[:, 0]

    def _need_speed(self, dist, passMode):
        hub = self.hubV[self.rtype]
        pas = self.passV[self.rtype]
        def look(t, d):
            i = (d / 0.5).clamp(0, t.shape[-1] - 1.001)
            i0 = i.floor().long()
            fr = i - i0
            v0 = t.gather(-1, i0[..., None]).squeeze(-1)
            v1 = t.gather(-1, (i0 + 1).clamp(max=t.shape[-1] - 1)[..., None]).squeeze(-1)
            v = v0 + (v1 - v0) * fr
            return torch.where(d / 0.5 > t.shape[-1] - 1, torch.nan, v)
        return torch.where(passMode, look(pas, dist), look(hub, dist))

    def _outtake(self, out, h, c, sn, hl):
        for r in range(2):
            f = out[:, r] & (self.rand(self.n) < h / 0.08)
            rows = f.nonzero().squeeze(-1)
            if len(rows) == 0:
                continue
            mine = (self.bst == HELD) & (self.bown == r)
            jj = mine.long().argmax(1)[rows]
            fx, fz = c[rows, r], -sn[rows, r]
            p = self.pos[rows, r] + torch.stack([fx, fz], -1) * (hl[rows, r] + 0.25)[:, None]
            self.bst[rows, jj] = FIELD
            self.bpos[rows, jj] = p
            self.bvel[rows, jj] = self.vel[rows, r] + torch.stack([fx, fz], -1) * 2.8
            self.stored[rows, r] -= 1

    def _arrive(self, arrive, act):
        auto, gap, tele, post, tt = self.phase()
        k = self.bkind
        a = self.bown
        hit = arrive & (k == FLY_HIT)
        if hit.any():
            actb = act.gather(1, a)                                     # [N,B]
            graceb = (self.t[:, None] - self.lastDeact.gather(1, a)) <= self.grace
            counts = hit & (actb | graceb) & ~(post[:, None] & ~graceb)
            autoP = auto[:, None] | (gap[:, None] & graceb)
            for al in (0, 1):
                m = counts & (a == al)
                self.score[:, al, 0] += (m & autoP).sum(1).float()
                self.score[:, al, 1] += (m & ~autoP).sum(1).float()
                self.score[:, al, 2] += (hit & ~(actb | graceb) & (a == al)).sum(1).float()
            self.bst = torch.where(hit, HUBQ, self.bst)
            self.btim = torch.where(hit, 0.45 + 0.85 * self.rand(self.n, self.B), self.btim)
        miss = arrive & (k == FLY_MISS)
        if miss.any():
            ang = self.rand(self.n, self.B) * 2 * math.pi
            r = self.hs + 0.15 + 0.8 * self.rand(self.n, self.B)
            p = self.bland + torch.stack([ang.cos(), ang.sin()], -1) * r[..., None]
            self.bpos = torch.where(miss[..., None], p, self.bpos)
            self.bvel = torch.where(miss[..., None], torch.stack([ang.cos(), ang.sin()], -1) * 0.8, self.bvel)
            self.bst = torch.where(miss, FIELD, self.bst)
        land = arrive & (k == FLY_LAND)
        if land.any():
            self.bpos = torch.where(land[..., None], self.bland, self.bpos)
            self.bvel = torch.where(land[..., None], self.randn(self.n, self.B, 2) * 0.4, self.bvel)
            self.bst = torch.where(land, FIELD, self.bst)

    def _exit_hub(self, ex):
        a = self.bown
        s = torch.where(a == 0, 1.0, -1.0)                              # toward the NEUTRAL ZONE
        off = (self.rand(self.n, self.B) - 0.5) * (self.exitW - 2 * self.R)   # across the opening
        ang = off / (self.exitW / 2) * 0.5 + (self.rand(self.n, self.B) - 0.5) * 0.8
        sp = self.exitSpeed[0] + (self.exitSpeed[1] - self.exitSpeed[0]) * self.rand(self.n, self.B)
        # leaves the raised opening and drops ~0.8 m: lands about 0.45 s later, a bit slower
        vx, vz = s * ang.cos() * sp, s * ang.sin() * sp
        x = torch.where(a == 0, self.hubX, -self.hubX) + s * (self.hs + self.R + 0.03) + vx * 0.4
        z = s * off + vz * 0.4
        self.bpos = torch.where(ex[..., None], torch.stack([x, z], -1), self.bpos)
        self.bvel = torch.where(ex[..., None], torch.stack([vx, vz], -1) * 0.8, self.bvel)
        self.bst = torch.where(ex, FIELD, self.bst)

    # ------------------------------------------------------------------ observation (= js/nn/obs.js)
    def obs(self):
        """[N,2,D] observation of each robot, built like buildObs() in js/nn/obs.js."""
        N, d = self.n, self.dev
        s = self.sgn
        parts = []
        P = lambda *xs: parts.append(torch.stack(xs, -1) if len(xs) > 1 else xs[0][..., None])
        x, z = s * self.pos[..., 0], s * self.pos[..., 1]
        yaw = self.yaw + torch.where(s > 0, 0.0, math.pi)
        c, sn = yaw.cos(), yaw.sin()
        def toLocal(dx, dz):
            return dx * c - dz * sn, dx * sn + dz * c
        turret = self.spec('turret')
        n_ = self.stored
        cap = self.spec('capMax')
        capNow = torch.floor(self.spec('capIn') + (self.spec('capOut') - self.spec('capIn')) * self.hopper + 1e-6)
        inZ = self.in_zone(s)
        P(x / self.HL, z / self.HW, c, sn, s * self.vel[..., 0] / 5, s * self.vel[..., 1] / 5, self.omega / 10,
          n_ / 60, n_ / cap, capNow / cap, self.deploy, self.hopper, self.fly / self.spec('speedMax'),
          self.ready.float(), self.shot.float(), inZ.float(), self.feeding.float(),
          torch.where(turret, self.turret / math.pi, torch.zeros_like(x)))
        parts.append(torch.nn.functional.one_hot(self.rtype, self.NR).float())
        P(self.spec('maxSpeed') / 5, self.spec('bps') / 40, turret.float())
        # hubs (own, then the other) and the zone line
        for own in (True, False):
            hx = torch.full_like(x, self.hubX) * (1 if own else -1)    # own frame: own HUB at hubX
            lx, lz = toLocal(hx - x, -z)
            P(lx / 8, lz / 8, torch.hypot(lx, lz) / 8)
        P((self.lineX - x) / self.HL)
        # match
        auto, gap, tele, post, tt = self.phase()
        pt = torch.where(auto, self.t, tt)
        left = torch.where(auto, self.TOTAL - self.t, torch.where(tele, self.T_TELE - tt, torch.where(gap, torch.full_like(tt, self.T_TELE), torch.zeros_like(tt))))
        ph = torch.where(auto, (self.T_AUTO - self.t) / self.T_AUTO, torch.where(tele, (self.T_TELE - tt) / self.T_TELE, torch.zeros_like(tt)))
        si = self.shift_index(tele, tt)
        m = torch.stack([auto.float(), tele.float(), left / self.TOTAL, ph], -1)[:, None].expand(N, 2, 4)
        parts.append(m)
        oh = torch.stack([(si == k).float() for k in range(6)], -1)[:, None].expand(N, 2, 6)
        parts.append(oh)
        act = self.hub_active()
        al = self.alliance_idx()
        nc = self.next_change()
        g = lambda t2, i: t2.gather(1, i)
        tot = self.totals()
        mine, theirs = g(tot, al), g(tot, 1 - al)
        P(g(act.float(), al), g(act.float(), 1 - al), g(nc, al), g(nc, 1 - al), torch.tanh((mine - theirs) / 100), mine / 300)
        # the other robot
        o = torch.tensor([1, 0], device=d)
        fpres = self.present[:, o]
        fx, fz = s * self.pos[:, o, 0], s * self.pos[:, o, 1]
        lx, lz = toLocal(fx - x, fz - z)
        fyaw = self.yaw[:, o] + torch.where(s > 0, 0.0, math.pi)
        ext = self.spec('halfL') * self.yaw.cos().abs() + self.spec('halfW') * self.yaw.sin().abs()
        foeInMine = (s * self.pos[:, o, 0] - ext[:, o]) <= self.lineX
        foeInOwn = (self.sgn[:, o] * self.pos[:, o, 0] - ext[:, o]) <= self.lineX
        fo = torch.stack([torch.ones_like(x), fx / self.HL, fz / self.HW, lx / 8, lz / 8, torch.hypot(lx, lz) / 8,
                          s * self.vel[:, o, 0] / 5, s * self.vel[:, o, 1] / 5, fyaw.cos(), fyaw.sin(), self.stored[:, o] / 60], -1)
        fo = torch.cat([fo, torch.nn.functional.one_hot(self.rtype[:, o], self.NR).float(),
                        torch.stack([foeInMine.float(), foeInOwn.float()], -1)], -1)
        parts.append(fo * fpres[..., None].float())
        # rays (robot frame, from the center) against walls and field elements
        parts.append(self._rays(yaw, s) / 4.0)
        # FUEL
        parts.append(self._fuel_obs(x, z, c, sn, s))
        out = torch.cat(parts, -1)
        return out

    def _rays(self, yaw_own, s):
        N = self.n
        k = torch.arange(16, device=self.dev).float() / 16 * 2 * math.pi
        ang = yaw_own[..., None] + k                     # [N,2,16]
        dx, dz = ang.cos() * s[..., None], -ang.sin() * s[..., None]
        px, pz = self.pos[..., 0, None], self.pos[..., 1, None]
        best = torch.full_like(dx, 4.0)
        eps = 1e-6
        tx = torch.where(dx > eps, (self.HL - px) / dx.clamp(min=eps), torch.where(dx < -eps, (-self.HL - px) / dx.clamp(max=-eps), torch.full_like(dx, 1e9)))
        tz = torch.where(dz > eps, (self.HW - pz) / dz.clamp(min=eps), torch.where(dz < -eps, (-self.HW - pz) / dz.clamp(max=-eps), torch.full_like(dz, 1e9)))
        best = torch.minimum(best, torch.minimum(tx, tz))
        R = self.obs_rects
        lo_x, hi_x = (R[:, 0] - R[:, 2]), (R[:, 0] + R[:, 2])
        lo_z, hi_z = (R[:, 1] - R[:, 3]), (R[:, 1] + R[:, 3])
        def slab(p, d, lo, hi):
            pe, de = p[..., None], d[..., None]
            par = de.abs() < 1e-9
            ta = (lo - pe) / torch.where(par, torch.ones_like(de), de)
            tb = (hi - pe) / torch.where(par, torch.ones_like(de), de)
            t0, t1 = torch.minimum(ta, tb), torch.maximum(ta, tb)
            outside = par & ((pe < lo) | (pe > hi))
            t0 = torch.where(par, torch.full_like(t0, -1e9), t0)
            t1 = torch.where(par, torch.full_like(t1, 1e9), t1)
            return t0, t1, outside
        a0, a1, ao = slab(px, dx, lo_x, hi_x)
        b0, b1, bo = slab(pz, dz, lo_z, hi_z)
        t0 = torch.maximum(torch.maximum(a0, b0), torch.zeros_like(a0))
        t1 = torch.minimum(torch.minimum(a1, b1), best[..., None])
        hit = (t0 <= t1) & ~ao & ~bo
        th = torch.where(hit, t0, torch.full_like(t0, 1e9)).min(-1).values
        best = torch.minimum(best, th)
        return best.clamp(min=0)

    def _fuel_obs(self, x, z, c, sn, s):
        N, d = self.n, self.dev
        onf = (self.bst == FIELD)[:, None, :].expand(N, 2, self.B)
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
        ego = torch.zeros(N, 2, EN * EN + 1, device=d).scatter_add_(-1, ei, torch.ones_like(lx))[..., :EN * EN]
        ego = (ego / 4).clamp(max=1)
        GX, GZ = 16, 8
        gx = ((bx + self.HL) / (2 * self.HL) * GX).floor().long()
        gz = ((bz + self.HW) / (2 * self.HW) * GZ).floor().long()
        ing = onf & (gx >= 0) & (gx < GX) & (gz >= 0) & (gz < GZ)
        gi = torch.where(ing, gx.clamp(0, GX - 1) * GZ + gz.clamp(0, GZ - 1), torch.full_like(gx, GX * GZ))
        grid = torch.zeros(N, 2, GX * GZ + 1, device=d).scatter_add_(-1, gi, torch.ones_like(lx))[..., :GX * GZ]
        grid = torch.log1p(grid) / math.log1p(20)
        d2 = torch.where(onf, lx * lx + lz * lz, torch.full_like(lx, 1e9))
        vals, ii = torch.topk(d2, 8, dim=-1, largest=False)
        nl = lx.gather(-1, ii)
        nz = lz.gather(-1, ii)
        ok = vals < 1e8
        near = torch.stack([torch.where(ok, (nl / 4).clamp(-1, 1), torch.ones_like(nl)), torch.where(ok, (nz / 4).clamp(-1, 1), torch.zeros_like(nz))], -1).reshape(N, 2, 16)
        line = self.lineX
        zo = (onf & (bx < line)).sum(-1).float() / 50
        zm = (onf & (bx >= line) & (bx <= -line)).sum(-1).float() / 400
        zp = (onf & (bx > -line)).sum(-1).float() / 50
        return torch.cat([ego, grid, near, torch.stack([zo, zm, zp], -1)], -1)

    # ------------------------------------------------------------------ scripted opponent
    def bot_actions(self, r=1):
        """A simple scripted Scorer for robot r: collect the nearest FUEL, score in its zone
        while its HUB is active. Returns [N,7] actions (own frame)."""
        N, d = self.n, self.dev
        s = self.sgn[:, r]
        x, z = s * self.pos[:, r, 0], s * self.pos[:, r, 1]
        yaw = self.yaw[:, r] + torch.where(s > 0, 0.0, math.pi)
        al = self.alliance_idx()[:, r]
        act = self.hub_active().gather(1, al[:, None]).squeeze(1)
        nc = self.next_change().gather(1, al[:, None]).squeeze(1) * self.T_SHIFT
        cap = self.spec('capMax')[:, r]
        n_ = self.stored[:, r]
        auto, gap, tele, post, tt = self.phase()
        go_score = (n_ >= 0.75 * cap) | ((n_ >= 4) & (act | (nc < 4))) | (auto & (n_ > 0) & (self.t > 12))
        # nearest FUEL
        onf = self.bst == FIELD
        bx, bz = s[:, None] * self.bpos[..., 0], s[:, None] * self.bpos[..., 1]
        d2 = torch.where(onf, (bx - x[:, None]) ** 2 + (bz - z[:, None]) ** 2, torch.full_like(bx, 1e9))
        j = d2.argmin(1)
        tx = torch.where(go_score, torch.full_like(x, self.lineX - 1.2), bx.gather(1, j[:, None]).squeeze(1))
        tz = torch.where(go_score, torch.where(z > 0, 1.7, -1.7), bz.gather(1, j[:, None]).squeeze(1))
        # repulsion from field elements (own frame: they're symmetric)
        dx, dz = tx - x, tz - z
        dist = torch.hypot(dx, dz).clamp(min=1e-3)
        vx, vz = dx / dist, dz / dist
        Rr = self.obs_rects
        rx = (x[:, None] - Rr[:, 0]).abs() - Rr[:, 2]
        rz = (z[:, None] - Rr[:, 1]).abs() - Rr[:, 3]
        gap_ = torch.maximum(rx, rz)
        close = gap_ < 0.9
        px = torch.where(close, (x[:, None] - Rr[:, 0]).sign() * (rx > rz).float() / (gap_.clamp(min=0.05)), 0.0).sum(1)
        pz = torch.where(close, (z[:, None] - Rr[:, 1]).sign() * (rz >= rx).float() / (gap_.clamp(min=0.05)), 0.0).sum(1)
        vx, vz = vx + 0.25 * px, vz + 0.25 * pz
        nv = torch.hypot(vx, vz).clamp(min=1e-3)
        sp = torch.where(go_score, 1.0, (dist / 1.5).clamp(0.4, 1.0)) * self.bot_skill
        out = torch.zeros(N, 7, device=d)
        out[:, 0], out[:, 1] = vx / nv * sp, vz / nv * sp
        want = torch.atan2(-dz, dx)
        out[:, 2] = (wrap(want - yaw) * 2).clamp(-1, 1) * (~go_score).float()
        out[:, 3] = (~go_score).float()
        inZ = self.in_zone(self.sgn)[:, r]
        out[:, 4] = (go_score & inZ & (act | (nc < 1.0))).float()
        return out
