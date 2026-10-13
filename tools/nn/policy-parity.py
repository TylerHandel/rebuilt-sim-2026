#!/usr/bin/env python3
"""Check that the attention + memory network (tools/nn/models.py) gives the same answers in the
game's JavaScript (js/nn/policy.js) as in PyTorch: a network with random weights drives a few
robots for a few seconds of GPU-simulator observations (one match restarting part way through,
so its memory resets), in both, step by step.

  python tools/nn/policy-parity.py          # prints the largest difference; fails above 1e-3
"""
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).parent))
from models import AttnGruActor  # noqa: E402
from gpusim import GpuSim  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


class Norm:
    def __init__(self, mean, std):
        self.mean, self.std = mean, std


def main():
    torch.manual_seed(3)
    P = json.loads((Path(__file__).with_name('gpusim-params.json')).read_text())
    sim = GpuSim(4, device='cpu', seed=5)
    sim.set_mix([0.0, 1.0, 0.0, 0.0])
    sim.reset(torch.ones(4, dtype=torch.bool))
    T, rows = 30, [0, 1, 6 + 0, 6 + 3]     # two robots of match 0, two of match 1 (one per alliance)
    obs, resets = [], []
    for t in range(T):
        o = sim.obs().view(-1, sim.D)
        obs.append(o[rows].clone())
        _, done = sim.step(sim.bot_actions())
        done[1] = done[1] | (t == 14)           # match 1 restarts after step 14
        sim.reset(done)
        resets.append(done.repeat_interleave(6)[rows].float())
    raw = torch.stack(obs)                      # [T,R,D]
    m = AttnGruActor(sim.D, P['obs']['layout'], P['field'])
    with torch.no_grad():                       # livelier than a fresh init: larger biases and heads
        for name, p in m.named_parameters():
            if name.endswith('bias') or name.startswith(('k_null', 'v_null', 'bk_', 'bv_')):
                p.add_(torch.randn_like(p) * 0.3)
        m.mu.weight.mul_(100)
        m.lg.weight.mul_(100)
    flat = raw.view(-1, sim.D)
    mean, std = flat.mean(0), flat.std(0) + 0.05
    x = ((raw - mean) / std).clamp(-10, 10)
    with torch.no_grad():
        mu, lg = m.seq(x, raw, m.initial_state(len(rows)), torch.stack(resets))
    tmp = Path(tempfile.mkdtemp())
    try:
        m.export(Norm(mean.numpy(), std.numpy()), tmp / 'net.json', 1, {}, P['obs']['version'])
        (tmp / 'obs.json').write_text(json.dumps({'obs': raw.tolist(), 'resets': torch.stack(resets).tolist()}))
        js = f"""
import fs from 'node:fs';
import {{ Policy }} from {json.dumps((ROOT / 'js' / 'nn' / 'policy.js').as_uri())};
const pol = new Policy(JSON.parse(fs.readFileSync({json.dumps(str(tmp / 'net.json'))}, 'utf8')));
const d = JSON.parse(fs.readFileSync({json.dumps(str(tmp / 'obs.json'))}, 'utf8'));
const R = d.obs[0].length, states = Array.from({{ length: R }}, () => pol.newState()), out = [];
for (let t = 0; t < d.obs.length; t++) {{
  out.push(d.obs[t].map((o, r) => {{ const f = pol.forward(Float32Array.from(o), states[r]); return [...f.mu, ...f.logits]; }}));
  d.resets[t].forEach((z, r) => {{ if (z) states[r].fill(0); }});
}}
process.stdout.write(JSON.stringify(out));
"""
        res = subprocess.run(['node', '--import', (ROOT / 'tools' / 'node-env.mjs').as_uri(), '--input-type=module', '-e', js],
                             cwd=str(ROOT), capture_output=True, text=True)
        if res.returncode:
            sys.exit(res.stderr)
        out = torch.tensor(json.loads(res.stdout))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    ref = torch.cat([mu, lg], -1)
    diff = (out - ref).abs().max().item()
    print(f'{T} decisions x {len(rows)} robots, outputs up to {ref.abs().max().item():.2f}: largest difference PyTorch vs JavaScript {diff:.2e}')
    if diff > 1e-3:
        sys.exit('MISMATCH')
    print('OK')


if __name__ == '__main__':
    main()
