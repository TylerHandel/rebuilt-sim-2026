// The js/trainedBrain.js file format, shared by the browser exporter (AI Tuning) and the
// Node trainer, plus a parser that accepts that file or plain JSON.
import { BRAIN_SPEC } from './opponent.js';

export function brainFileText(brain, info = null) {
  const body = Object.keys(BRAIN_SPEC)
    .map((k) => `  ${k}: ${+(+brain[k]).toFixed(3)}, // ${BRAIN_SPEC[k].desc}`)
    .join('\n');
  return `// Strategy learned by AI training. Written by tools/train.mjs or exported from the in-game\n// AI Tuning screen; replace this file to change the shipped "Trained" AI.\nexport const TRAINED_BRAIN = {\n${body}\n};\n\nexport const TRAINING_INFO = ${JSON.stringify(info, null, 2)};\n`;
}

// Accepts exported JSON ({ brain: {...} } or a bare brain object) or a trainedBrain.js file.
// Returns only known, in-range brain values.
export function parseBrainText(text) {
  let obj = null;
  try {
    const j = JSON.parse(text);
    obj = j && typeof j === 'object' ? (j.brain && typeof j.brain === 'object' ? j.brain : j) : null;
  } catch {
    const m = /TRAINED_BRAIN\s*=\s*\{([\s\S]*?)\};/.exec(text);
    if (m) {
      obj = {};
      for (const [, k, v] of m[1].matchAll(/(\w+)\s*:\s*(-?[\d.]+(?:e-?\d+)?)/g)) obj[k] = +v;
    }
  }
  if (!obj) return null;
  const out = {};
  for (const [k, spec] of Object.entries(BRAIN_SPEC)) {
    const v = +obj[k];
    if (Number.isFinite(v)) out[k] = Math.min(spec.max, Math.max(spec.min, v));
  }
  return Object.keys(out).length ? out : null;
}
