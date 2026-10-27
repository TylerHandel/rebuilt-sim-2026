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

// Squeezed FUEL flattens where it's pressed: up to four contact patches per instance (touch0..3:
// the patch's normal, in the instance's own frame, and its distance from the center). Foam gives
// around a contact too, not just on it, so the sphere eases into each flat patch over a band that
// widens the harder it's pressed (a smooth minimum of the sphere and the contact plane). Adds the
// attributes to geo (n instances, each with no patch: a plane beyond the ball) and the squash to
// mat; returns the four attributes.
export function squashable(geo, mat, n, r) {
  const touch = [0, 1, 2, 3].map((j) => {
    const a = new THREE.InstancedBufferAttribute(new Float32Array(4 * n), 4);
    for (let i = 0; i < n; i++) { a.array[4 * i + 1] = 1; a.array[4 * i + 3] = 1; }
    a.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('touch' + j, a);
    return a;
  });
  mat.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', `#include <common>
attribute vec4 touch0; attribute vec4 touch1; attribute vec4 touch2; attribute vec4 touch3;
void squash(vec4 c, inout vec3 p, inout vec3 nrm) {
  float k = 0.01 + 1.2 * max(0.0, ${r.toFixed(4)} - c.w); // blend band: wider for a deeper squash
  float h = dot(p, c.xyz);
  float q = max(k - abs(h - c.w), 0.0) / k;
  float hs = min(h, c.w) - q * q * k * 0.25;
  if (hs < h) {
    p -= c.xyz * (h - hs);
    nrm = normalize(mix(c.xyz, nrm, clamp(0.5 + 0.5 * (c.w - h) / k, 0.0, 1.0)));
  }
}`)
      .replace('#include <beginnormal_vertex>', `#include <beginnormal_vertex>
vec3 sqP = position;
squash(touch0, sqP, objectNormal); squash(touch1, sqP, objectNormal);
squash(touch2, sqP, objectNormal); squash(touch3, sqP, objectNormal);`)
      .replace('#include <begin_vertex>', 'vec3 transformed = sqP;');
  };
  return touch;
}
