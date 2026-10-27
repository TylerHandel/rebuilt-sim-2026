import * as THREE from 'three';

// FUEL's skin, wrapped round the sphere (longitude across, latitude down): the lemon yellow of
// the official ball (AndyMark's photos: about sRGB 233, 206, 1), its molding's mid seam, a black
// print on one side and a date code on the other (the real ball's are the FIRST and AndyMark
// logos and a three-digit Julian date above the seam), so its spin shows from any side
let skin = null;
export function fuelTexture() {
  if (skin) return skin;
  const W = 512, H = 256, c = document.createElement('canvas');
  c.width = W; c.height = H;
  const g = c.getContext('2d');
  g.fillStyle = '#e9ce02';
  g.fillRect(0, 0, W, H);
  // the seam round the middle
  g.fillStyle = '#c4a900';
  g.fillRect(0, H / 2 - 1.5, W, 3);
  g.fillStyle = '#161616';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  // the print: two lines, the upper bolder (stretched a little as it nears the pole)
  g.font = 'italic 900 30px Arial, sans-serif';
  g.save(); g.translate(W * 0.25, H * 0.39); g.scale(1.25, 1); g.fillText('FUEL', 0, 0); g.restore();
  g.font = '700 18px Arial, sans-serif';
  g.save(); g.translate(W * 0.25, H * 0.6); g.scale(1.1, 1); g.fillText('REBUILT 2026', 0, 0); g.restore();
  // the date code, small, just above the seam on the far side
  g.font = '700 16px Arial, sans-serif';
  g.fillText('026', W * 0.75, H * 0.44);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return (skin = t);
}
