"""The 'attn-gru' driving network: attention over the objects on the field, and memory.

It reads the same observation as the plain MLP (js/nn/obs.js, v4), split into:
  * context: its own robot, both HUBs, the match clock and score, the 16 distance rays, the FUEL
    map around it (12 x 12 cells) and the zone counts -> an MLP;
  * objects, as tokens it attends to: the other 5 robots (nearest opponent, both teammates, the
    other two opponents), the 128 cells of the whole-field FUEL map (where they are relative to
    it, and how much FUEL), and the 8 nearest FUEL. A multi-head attention, asked by the context,
    picks out what matters right now (a pile, a defender closing in, a teammate on the same pile);
  * memory: a GRU carries what it has seen from one decision (0.1 s) to the next, so it can tell
    which way an opponent is heading, whether a defender keeps following it, what it was doing.
Then a head gives the drive / turn means and the button logits, like the MLP.

Cheap by construction: every token kind has a linear key / value map, so the attention never
builds per-token keys and values (q.k_i = (W_k^T q).t_i, sum_i a_i v_i = W_v (sum_i a_i t_i)): the
same result as ordinary attention at a fraction of the memory. It runs in the browser too
(js/nn/policy.js, the same math in plain JavaScript; tools/nn/policy-parity.py checks they agree).
"""
import base64
import json
import math
import os
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

ACT_CONT, ACT_BIN = 3, 4
GX, GZ = 16, 8          # whole-field FUEL map (obs.js)
NEAR = 8                # nearest FUEL listed
ROBOT_BLOCK = 24        # a robot as seen by another (obs.js robotBlock)
EXTRA = 8               # one-hot slots for robots added since v3 (robotsNew)


SIZES = {
    # ~0.8M weights: ~1.5 ms per robot per decision in the browser
    'base': {'ctx': [256, 256], 'heads': 4, 'dh': 32, 'remb': 64, 'att': 128, 'gru': 256, 'head': [256]},
    # ~3.1M weights: ~4x the compute (for the full-game trainer, where the GPU has time to spare)
    'large': {'ctx': [512, 512], 'heads': 8, 'dh': 32, 'remb': 128, 'att': 256, 'gru': 512, 'head': [512]},
}


def default_cfg():
    return dict(SIZES['base'])


def obs_slices(layout):
    """name -> (start, end) in the observation, from gpusim-params.json's obs.layout."""
    out, o = {}, 0
    for name, n in layout:
        out[name] = (o, o + n)
        o += n
    return out


class AttnGruActor(nn.Module):
    recurrent = True

    def __init__(self, obs_dim, layout, field, cfg=None):
        super().__init__()
        cfg = {**default_cfg(), **(cfg or {})}
        self.cfg, self.layout, self.field = cfg, [list(x) for x in layout], dict(field)
        S = obs_slices(layout)
        self.obs_dim = obs_dim
        # the context: everything but the other robots, the field map and the nearest FUEL
        ctx_idx = [*range(*S['self']), *range(*S['hubs']), *range(*S['match']), *range(*S['rays']), *range(*S['fuelEgo']),
                   *range(*S['fuelZones']), *range(S['robotsNew'][0], S['robotsNew'][0] + EXTRA)]
        self.register_buffer('ctx_idx', torch.tensor(ctx_idx), persistent=False)
        # the other robots: block start, its v4 one-hot start, teammate?
        rb = [S['opponent'][0], S['teammates'][0], S['teammates'][0] + ROBOT_BLOCK, S['opponents2'][0], S['opponents2'][0] + ROBOT_BLOCK]
        nw = [S['robotsNew'][0] + EXTRA * k for k in range(1, 6)]
        self.register_buffer('rob_idx', torch.tensor([[*range(b, b + ROBOT_BLOCK), *range(w, w + EXTRA)] for b, w in zip(rb, nw)]), persistent=False)
        self.register_buffer('rob_mate', torch.tensor([0.0, 1.0, 1.0, 0.0, 0.0]), persistent=False)
        self.register_buffer('rob_on', torch.tensor(rb), persistent=False)
        self.f0, self.n0 = S['fuelField'][0], S['fuelNear'][0]
        gx = torch.arange(GX).repeat_interleave(GZ).float()
        gz = torch.arange(GZ).repeat(GX).float()
        self.register_buffer('cell_c', torch.stack([(gx + 0.5) / GX * 2 - 1, (gz + 0.5) / GZ * 2 - 1], -1), persistent=False)  # [128,2] x/HL, z/HW
        H, dh, D = cfg['heads'], cfg['dh'], cfg['att']
        self.H, self.dh = H, dh
        layers, n = [], len(ctx_idx)
        for h in cfg['ctx']:
            layers += [nn.Linear(n, h), nn.Tanh()]
            n = h
        self.ctx = nn.Sequential(*layers)
        C = n
        self.q = nn.Linear(C, H * dh)
        self.remb = nn.Linear(ROBOT_BLOCK + EXTRA + 1, cfg['remb'])
        # per token kind: key / value maps [H, dh, features]
        self.kinds = {'robot': cfg['remb'], 'cell': 6, 'near': 3}
        for k, f in self.kinds.items():
            self.register_parameter(f'wk_{k}', nn.Parameter(torch.randn(H, dh, f) / math.sqrt(f)))
            self.register_parameter(f'bk_{k}', nn.Parameter(torch.zeros(H, dh)))
            self.register_parameter(f'wv_{k}', nn.Parameter(torch.randn(H, dh, f) / math.sqrt(f)))
            self.register_parameter(f'bv_{k}', nn.Parameter(torch.zeros(H, dh)))
        self.k_null = nn.Parameter(torch.zeros(H, dh))
        self.v_null = nn.Parameter(torch.zeros(H, dh))
        self.out = nn.Linear(H * dh, D)
        self.gru = nn.GRUCell(C + D, cfg['gru'])
        layers, n = [], cfg['gru'] + C
        for h in cfg['head']:
            layers += [nn.Linear(n, h), nn.Tanh()]
            n = h
        self.head = nn.Sequential(*layers)
        self.mu = nn.Linear(n, ACT_CONT)
        self.lg = nn.Linear(n, ACT_BIN)
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, math.sqrt(2))
                nn.init.zeros_(m.bias)
        nn.init.orthogonal_(self.q.weight, 1.0)
        nn.init.orthogonal_(self.mu.weight, 0.01)
        nn.init.orthogonal_(self.lg.weight, 0.01)
        with torch.no_grad():
            self.lg.bias.copy_(torch.tensor([1.0, -1.0, -2.5, -3.5]))  # holding the intake, not shooting
        self.log_std = nn.Parameter(torch.full((ACT_CONT,), -0.5))
        self.hidden = [cfg['gru']]  # (for logs)

    @property
    def hdim(self):
        return self.cfg['gru']

    def initial_state(self, n, device=None):
        return torch.zeros(n, self.cfg['gru'], device=device)

    def tokens(self, x, raw):
        """The objects: (features [R,n,f], usable [R,n]) per kind."""
        HL, HW = self.field['halfL'], self.field['halfW']
        R = x.shape[0]
        rob = x.index_select(1, self.rob_idx.view(-1)).view(R, 5, -1)
        rob = torch.cat([rob, self.rob_mate.expand(R, 5)[..., None]], -1)
        rob_ok = raw.index_select(1, self.rob_on) > 0.5
        sx, sz, c, sn = raw[:, 0:1], raw[:, 1:2], raw[:, 2:3], raw[:, 3:4]
        dxm = (self.cell_c[:, 0] - sx) * HL
        dzm = (self.cell_c[:, 1] - sz) * HW
        lx, lz = dxm * c - dzm * sn, dxm * sn + dzm * c
        v = raw[:, self.f0:self.f0 + GX * GZ]
        cell = torch.stack([self.cell_c[:, 0].expand(R, -1), self.cell_c[:, 1].expand(R, -1), lx / 8, lz / 8, torch.sqrt(lx * lx + lz * lz + 1e-9) / 8, v], -1)
        nr = raw[:, self.n0:self.n0 + 2 * NEAR].view(R, NEAR, 2)
        near = torch.cat([nr, torch.sqrt((nr * nr).sum(-1, keepdim=True) + 1e-9)], -1)
        near_ok = ~((nr[..., 0] == 1) & (nr[..., 1] == 0))
        return {'robot': (torch.tanh(self.remb(rob)), rob_ok), 'cell': (cell, v > 0), 'near': (near, near_ok)}

    def step(self, x, raw, h):
        """One decision for R robots: x normalized observation [R,D], raw [R,D], h memory [R,G]
        -> drive / turn means [R,3], button logits [R,4], new memory [R,G]."""
        R = x.shape[0]
        ctx = self.ctx(x.index_select(1, self.ctx_idx))
        q = self.q(ctx).view(R, self.H, self.dh) / math.sqrt(self.dh)
        toks = self.tokens(x, raw)
        scores, parts = [], []
        for k, (t, ok) in toks.items():
            wk, bk = getattr(self, f'wk_{k}'), getattr(self, f'bk_{k}')
            qk = torch.einsum('rhd,hdf->rhf', q, wk)
            s = torch.einsum('rhf,rnf->rhn', qk, t) + torch.einsum('rhd,hd->rh', q, bk)[..., None]
            scores.append(s.masked_fill(~ok[:, None, :], -1e9))
            parts.append((k, t, s.shape[-1]))
        scores.append(torch.einsum('rhd,hd->rh', q, self.k_null)[..., None])
        a = torch.softmax(torch.cat(scores, -1), -1)
        o = a[..., -1:] * self.v_null
        i = 0
        for k, t, n in parts:
            ak = a[..., i:i + n]
            i += n
            tbar = torch.einsum('rhn,rnf->rhf', ak, t)
            o = o + torch.einsum('hdf,rhf->rhd', getattr(self, f'wv_{k}'), tbar) + getattr(self, f'bv_{k}') * ak.sum(-1, keepdim=True)
        att = self.out(o.reshape(R, -1))
        h = self.gru_step(torch.cat([ctx, att], -1), h)
        z = self.head(torch.cat([h, ctx], -1))
        return self.mu(z), self.lg(z), h

    def gru_step(self, inp, h):
        """nn.GRUCell's math written out (gate order r, z, n), so it batches under torch.func.vmap
        (the older versions run all at once) and compiles."""
        g = self.gru
        gi = nn.functional.linear(inp, g.weight_ih, g.bias_ih)
        gh = nn.functional.linear(h, g.weight_hh, g.bias_hh)
        ir, iz, inn = gi.chunk(3, -1)
        hr, hz, hn = gh.chunk(3, -1)
        r, z = torch.sigmoid(ir + hr), torch.sigmoid(iz + hz)
        return (1 - z) * torch.tanh(inn + r * hn) + z * h

    def forward(self, x, raw, h):
        return self.step(x, raw, h)

    def spec(self):
        """What it takes to build it again (saved in checkpoints)."""
        return {'layout': self.layout, 'field': self.field, 'cfg': self.cfg}

    def seq(self, x, raw, h0, reset):
        """A rollout: x, raw [T,R,D], h0 [R,G], reset [T,R] (1: the match ended after step t, so
        memory starts empty at t+1) -> means [T,R,3], logits [T,R,4]."""
        mus, lgs, h = [], [], h0
        for t in range(x.shape[0]):
            mu, lg, h = self.step(x[t], raw[t], h)
            mus.append(mu)
            lgs.append(lg)
            h = h * (1.0 - reset[t])[:, None]
        return torch.stack(mus), torch.stack(lgs)

    # ---- export (js/nn/policy.js reads this)
    def export(self, norm, path, version, info, obs_version):
        names, blobs = [], []
        for name, p in self.named_parameters():
            names.append([name, list(p.shape)])
            blobs.append(p.detach().float().cpu().numpy().ravel())
        data = {
            'format': 'rebuilt-nn-policy', 'arch': 'attn-gru', 'obsVersion': obs_version, 'obsDim': self.obs_dim,
            'version': version, 'cfg': self.cfg, 'layout': self.layout, 'field': self.field, 'params': names,
            # where its inputs come from in the observation (so the game doesn't have to work it out again)
            'idx': {'ctx': self.ctx_idx.tolist(), 'robots': self.rob_idx.tolist(), 'robotOn': self.rob_on.tolist(), 'mate': self.rob_mate.tolist(),
                    'field': self.f0, 'near': self.n0, 'gx': GX, 'gz': GZ, 'nearN': NEAR},
            'hidden': [self.cfg['gru']], 'activation': 'tanh',
            'obsMean': [float(v) for v in norm.mean], 'obsStd': [float(v) for v in norm.std],
            'logStd': [float(v) for v in self.log_std.detach().clamp(-3.0, 0.5).cpu()],
            'weights': base64.b64encode(np.concatenate(blobs).astype('<f4').tobytes()).decode('ascii'), 'info': info,
        }
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(data))
        os.replace(tmp, path)


def from_policy_json(d):
    """An AttnGruActor from an exported policy .json (and its observation mean / std)."""
    m = AttnGruActor(d['obsDim'], d['layout'], d['field'], d['cfg'])
    w = np.frombuffer(base64.b64decode(d['weights']), dtype='<f4')
    i = 0
    params = dict(m.named_parameters())
    with torch.no_grad():
        for name, shape in d['params']:
            n = int(np.prod(shape))
            params[name].copy_(torch.from_numpy(w[i:i + n].copy()).view(shape))
            i += n
        m.log_std.copy_(torch.tensor(d['logStd']))
    return m, np.array(d['obsMean']), np.array(d['obsStd'])
