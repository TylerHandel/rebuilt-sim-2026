# Which colors in each robot's CAD are aluminum: its Onshape bill of materials lists every part's
# material and appearance, so a color most of whose parts (that have a material) are aluminum is
# drawn as aluminum (js/robotModels.js cadPart). Writes js/aluminumColors.js, and in the cache dir
# <team>-materials.json, each part's material and placements: a recipe's "bom" in
# tools/extract-parts.mjs, which marks the aluminum parts' material part by part (the color rule
# is for the GLBs made without one).
#   python3 -I tools/aluminum-colors.py [cache dir]
# Needs ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY in the environment. The documents are the ones
# cad/robots/<team>.json's "source" names (2910's is in tools/path-mesh-2910.py).
import json, os, re, subprocess, sys, glob

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, '.bom-cache')
os.makedirs(CACHE, exist_ok=True)
API = 'https://cad.onshape.com/api/v9'
EXTRA = {'2910': ('dfb391aac173a4555d00a5b5', 'w', '3dc64f602735252892b0e47b', '6c654da4eb6b1710fb0900bd')}
SHARE = 0.5  # a color is aluminum when at least this share of its parts with a material are


def get(path):
    out = subprocess.run(['curl', '-s', '--location-trusted', '-u', f"{os.environ['ONSHAPE_ACCESS_KEY']}:{os.environ['ONSHAPE_SECRET_KEY']}",
                          '-H', 'Accept: application/json', API + path], capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def doc_ref(team):
    if team in EXTRA: return EXTRA[team]
    src = json.load(open(f'{ROOT}/cad/robots/{team}.json')).get('source', '')
    ids = re.findall(r'(?:(workspace|version|element)\s+)?([0-9a-f]{24})', src)
    did = ids[0][1]
    wv = next(((k, i) for k, i in ids[1:] if k in ('workspace', 'version')), None)
    eid = next((i for k, i in ids[1:] if k == 'element'), None)
    if wv is None:
        wv = ('workspace', get(f'/documents/{did}')['defaultWorkspace']['id'])
    return did, 'w' if wv[0] == 'workspace' else 'v', wv[1], eid


def bom(team):
    f = f'{CACHE}/{team}.json'
    if not os.path.exists(f):
        did, kind, wv, eid = doc_ref(team)
        j = get(f'/assemblies/d/{did}/{kind}/{wv}/e/{eid}/bom?indented=false&generateIfAbsent=true')
        json.dump(j, open(f, 'w'))
    return json.load(open(f))


def assembly(team):
    f = f'{CACHE}/{team}-asm.json'
    if not os.path.exists(f):
        did, kind, wv, eid = doc_ref(team)
        j = get(f'/assemblies/d/{did}/{kind}/{wv}/e/{eid}')
        if 'rootAssembly' not in j: raise RuntimeError(j.get('message', 'no assembly definition'))
        json.dump(j, open(f, 'w'))
    return json.load(open(f))


def materials(team):
    # Per part: its name, Onshape part id, material and where each of its instances sits, for
    # tools/extract-parts.mjs (a recipe's "bom"), which decides aluminum part by part: a color is
    # too coarse (8793's cable carrier and 3D prints share the aluminum plates' slate). Names
    # repeat ("Part 1", "Side Plate"), so two more keys tell such parts apart: the part id, which a
    # glTF export carries on each node (PTC_onshape_metadata), and the placements: the BOM row
    # lists the part's occurrences (instance-id paths) and the assembly definition has each one's
    # transform (row-major, meters), whose translation is the part studio's origin in the robot,
    # where an export puts the part's node. (A document shared by link may refuse the assembly
    # definition: no placements then.)
    j = bom(team)
    try:
        asm = assembly(team)
        occ = {'.'.join(o['path']): o['transform'] for o in asm['rootAssembly']['occurrences']}
    except Exception:
        occ = {}
    H = {h['name']: h['id'] for h in j.get('headers', [])}
    out = []
    for r in j.get('rows', []):
        v = r['headerIdToValue']
        m = v.get(H.get('Material'))
        mat = (m.get('displayName') or m.get('id') or '') if isinstance(m, dict) else (m or '')
        at = [[round(t[i], 4) for i in (3, 7, 11)] for t in (occ.get(p) for p in r.get('relatedOccurrences', [])) if t]
        out.append({'name': v.get(H.get('Name')) or '', 'partId': r.get('itemSource', {}).get('partId', ''), 'material': mat.strip(), 'at': at})
    with open(f'{CACHE}/{team}-materials.json', 'w') as f:
        json.dump({'team': team, 'parts': out}, f, separators=(',', ':'))
    return out


def colors(team):
    j = bom(team)
    H = {h['name']: h['id'] for h in j.get('headers', [])}
    tally = {}
    for r in j.get('rows', []):
        v = r['headerIdToValue']
        m, a = v.get(H.get('Material')), v.get(H.get('Appearance'))
        name = (m.get('displayName') or m.get('id') or '') if isinstance(m, dict) else (m or '')
        c = a.get('color') if isinstance(a, dict) else None
        if not name or not c: continue
        key = (c['red'], c['green'], c['blue'])
        al = bool(re.search(r'alumin|6061|7075|5052|6063|2024', name, re.I))
        t = tally.setdefault(key, [0, 0])
        t[0] += al; t[1] += 1
    out = []
    for (r, g, b), (al, n) in sorted(tally.items(), key=lambda kv: -kv[1][0]):
        if al >= 2 and al / n >= SHARE:
            # (a light gray, by its chroma and lightness: raw or clear-anodized aluminum, drawn in
            # whatever gray the CAD likes; a dark gray: anodized dark (kept dark, but metal); colored:
            # anodized or powder-coated. The slate 67, 72, 77 is a CAD library's aluminum (all of it
            # aluminum in 1678's CAD; 8793's aluminum, the team): raw. Near-black is a black coat, left
            # as it is like colored. js/robotModels.js aluFinish judges a part's color the same way)
            chroma, light = (max(r, g, b) - min(r, g, b)) / 255, max(r, g, b) / 255
            finish = 'colored' if chroma >= 0.15 or light < 0.1 else 'raw' if light >= 0.45 or (r, g, b) == (67, 72, 77) else 'dark'
            out.append({'rgb': [r, g, b], 'parts': al, 'of': n, 'finish': finish})
    return out


teams = sorted(os.path.basename(f)[:-5] for f in glob.glob(f'{ROOT}/cad/robots/*.json'))
res = {}
for t in teams:
    try:
        res[t] = colors(t)
        mats = materials(t)
        print(t, [(c['rgb'], c['finish'], f"{c['parts']}/{c['of']}") for c in res[t]],
              f"materials: {sum(1 for m in mats if m['at'])}/{len(mats)} parts placed")
    except Exception as e:
        print(t, 'no BOM:', e)
with open(f'{ROOT}/js/aluminumColors.js', 'w') as f:
    f.write('// Which colors in each robot\'s CAD are aluminum (its Onshape bill of materials: a color most of\n'
            '// whose parts are aluminum), and its finish: raw (bare or clear-anodized: drawn as brushed\n'
            '// aluminum), dark (black-anodized) or colored (anodized or powder-coated, left as it is).\n'
            '// Generated by tools/aluminum-colors.py.\n')
    f.write('export const ALUMINUM_COLORS = ' + json.dumps({k: [{'rgb': c['rgb'], 'finish': c['finish']} for c in v] for k, v in res.items() if v}) + ';\n')
