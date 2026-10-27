// The alliance wall cut away where it hides your robot: when the line from the camera to the robot
// crosses an ALLIANCE WALL below where it can be seen through (a Driver Station's base under its
// glass, the TOWER WALL, the OUTPOST: field.wallSpans), the field's surfaces in a corridor round
// that line, near the wall, aren't drawn. Clipping planes on the field's materials (the drawn
// field and its CAD; never the robots or the FUEL): a fragment is cut only when it's inside all of
// them (clipIntersection): within the corridor's sides, top and bottom, short of the robot, and
// within reach of the wall. Looking over the base through the glass, nothing's cut.
import * as THREE from 'three';

const HALF_W_CUT = 0.75; // the corridor's half width round the line to the robot (m)
const HALF_H_CUT = 0.7;  // and half height
const BAND = 0.45;       // how far onto the field from the wall's plane it cuts (behind it, all of the way)

export class WallCutaway {
  constructor(renderer, field) {
    this.field = field;
    renderer.localClippingEnabled = true;
    this.planes = Array.from({ length: 6 }, () => new THREE.Plane());
    this.mats = new Set();
    this.active = false;
    this._off();
  }

  // the field's materials under root get the cut (the floor and the carpet, which nothing hides,
  // are left alone)
  register(root) {
    root.traverse((o) => {
      if (!o.isMesh || o.userData.floor) return;
      for (const m of [].concat(o.material)) {
        if (!m || this.mats.has(m)) continue;
        m.clippingPlanes = this.planes;
        m.clipIntersection = true;
        m.needsUpdate = true;
        this.mats.add(m);
      }
    });
  }

  // nothing cut: a plane everything's on the positive side of
  _off() {
    this.planes[0].set(new THREE.Vector3(0, 1, 0), 1e6);
    this.active = false;
  }

  update(cam, robot) {
    if (!robot) { this._off(); return; }
    const C = cam.position, R = robot.pos;
    // the wall the line to the robot crosses, if it crosses one where it can't be seen through
    let wall = null;
    for (const y of [0.12, 0.3, 0.5]) {
      for (const sp of this.field.wallSpans) {
        const dx = R.x - C.x;
        if (Math.abs(dx) < 1e-6) continue;
        const t = (sp.x - C.x) / dx;
        if (t <= 0 || t >= 1) continue;
        const z = C.z + (R.z - C.z) * t, yy = C.y + (y - C.y) * t;
        if (z >= sp.z0 && z <= sp.z1 && yy < sp.h) { wall = sp.x; break; }
      }
      if (wall !== null) break;
    }
    if (wall === null) { this._off(); return; }
    // the corridor, from the camera to the robot (aimed at its middle)
    const T = new THREE.Vector3(R.x, 0.3, R.z);
    const d = T.clone().sub(C).normalize();
    const side = new THREE.Vector3().crossVectors(d, new THREE.Vector3(0, 1, 0)).normalize();
    const up = new THREE.Vector3().crossVectors(side, d).normalize();
    const [a, b, c, e, f, g] = this.planes;
    // (a point's inside each plane where its distance is negative)
    a.set(side, -side.dot(C) - HALF_W_CUT);
    b.set(side.clone().negate(), side.dot(C) - HALF_W_CUT);
    c.set(up, -up.dot(C) - HALF_H_CUT);
    e.set(up.clone().negate(), up.dot(C) - HALF_H_CUT);
    // short of the robot (what's beyond it stays)
    f.set(d, -d.dot(T) + 0.2);
    // within BAND of that wall on the field's side (behind it, out to the camera, too)
    const s = Math.sign(wall) || 1; // the wall at +-HALF_L; the field toward -s
    g.set(new THREE.Vector3(-s, 0, 0), s * wall - BAND);
    this.active = true;
  }
}
