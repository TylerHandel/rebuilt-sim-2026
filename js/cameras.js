import * as THREE from 'three';
import { HALF_L, HALF_W, BLUE, DRIVER_STATIONS } from './constants.js';

// A driver at its driver station: standing right up against the glass (the polycarbonate sits in
// the middle of the 12 cm ALLIANCE WALL, 6 cm behind the FIELD edge; the eyes are about 12 cm
// behind it) at standing eye height, turning its head to keep its robot in view.
const DS_EYE_BACK = 0.06 + 0.12; // m behind the FIELD edge
const DS_EYE_H = 1.6;            // m above the carpet

export const CAMERA_MODES = ['driver', 'follow', 'chase', 'overhead', 'broadcast'];
export const CAMERA_NAMES = {
  driver: 'Driver Station',
  follow: 'Follow',
  chase: 'Robot Chase',
  overhead: 'Overhead',
  broadcast: 'Broadcast',
};

export class CameraRig {
  constructor(camera) {
    this.camera = camera;
    this.mode = 'driver';
    this.pos = new THREE.Vector3(0, 10, 10);
    this.look = new THREE.Vector3();
    this.alliance = BLUE;
    this.ds = 1;
    this.snap = true;
    this.flip = false; // turned around 180 degrees (right stick click)
  }

  toggleFlip() {
    this.flip = !this.flip;
    this.snap = true;
    return this.flip;
  }

  setMode(m) {
    this.mode = m;
    this.snap = true;
  }

  cycle(dir) {
    const i = CAMERA_MODES.indexOf(this.mode);
    this.setMode(CAMERA_MODES[(i + dir + CAMERA_MODES.length) % CAMERA_MODES.length]);
    return CAMERA_NAMES[this.mode];
  }

  // Which way "up" on the stick drives, field-relative: away from the camera, toward the robot (the
  // way you're looking at it), so forward is forward whichever camera you watch from, even a fixed
  // one the robot drives around in front of. Overhead: up the screen. Driver Station: field-centric,
  // as a real driver drives: straight downfield, away from its own ALLIANCE WALL, wherever the robot
  // is and wherever the driver turns to look (turned around: toward the wall). Returns a unit {x, z}.
  driveForward(robot) {
    const cam = this.camera;
    let x, z;
    if (this.mode === 'driver') return { x: (this.alliance === BLUE ? 1 : -1) * (this.flip ? -1 : 1), z: 0 };
    if (this.mode === 'overhead') { x = cam.up.x; z = cam.up.z; }
    else {
      x = (robot ? robot.pos.x : this.look.x) - cam.position.x;
      z = (robot ? robot.pos.z : this.look.z) - cam.position.z;
      // right under the camera: the way it looks instead
      if (Math.hypot(x, z) < 0.6) { x = this.look.x - cam.position.x; z = this.look.z - cam.position.z; }
    }
    const n = Math.hypot(x, z) || 1;
    return { x: x / n, z: z / n };
  }

  update(dt, robot) {
    if (this.mode === 'free') return; // externally positioned (debug / photo)
    const s = this.alliance === BLUE ? 1 : -1; // +X is "downfield" for blue
    const cam = this.camera;
    let p, l, fov = 55;
    const rp = robot ? robot.pos : new THREE.Vector3();
    // turned around, a shot is mirrored through the robot, so it keeps its angle and distance (a
    // fixed camera then watches from the robot's other side, moving with it)
    const c = rp;
    switch (this.mode) {
      case 'driver': {
        const fy = DRIVER_STATIONS[this.ds].fy;
        const z = s * (HALF_W - fy);
        p = new THREE.Vector3(-s * (HALF_L + DS_EYE_BACK), DS_EYE_H, z);
        // watching its robot (a little ahead of it, downfield, where it's headed most of the time)
        l = robot ? new THREE.Vector3(rp.x + s * 1.0, 0.25, rp.z) : new THREE.Vector3(-s * (HALF_L - 8.5), 0, z * 0.3);
        fov = 55;
        break;
      }
      case 'follow': {
        p = new THREE.Vector3(rp.x - s * 3.4, 2.9, rp.z);
        l = new THREE.Vector3(rp.x + s * 1.5, 0.3, rp.z);
        fov = 60;
        break;
      }
      case 'chase': {
        const f = robot ? robot.forward() : new THREE.Vector3(s, 0, 0);
        p = new THREE.Vector3(rp.x - f.x * 2.6, 1.9, rp.z - f.z * 2.6);
        l = new THREE.Vector3(rp.x + f.x * 2.5, 0.5, rp.z + f.z * 2.5);
        fov = 65;
        break;
      }
      case 'overhead': {
        p = new THREE.Vector3(-s * 1.2, 15.5, 0.001);
        l = new THREE.Vector3(0, 0, 0);
        fov = 52;
        break;
      }
      case 'broadcast':
      default: {
        p = new THREE.Vector3(0, 7.5, HALF_W + 7.5);
        l = new THREE.Vector3(rp.x * 0.35, 0, 0);
        fov = 50;
        break;
      }
    }
    // turned around: the same shot from the other side of the robot
    if (this.flip && this.mode !== 'overhead') {
      p.x = 2 * c.x - p.x; p.z = 2 * c.z - p.z;
      l.x = 2 * c.x - l.x; l.z = 2 * c.z - l.z;
    }
    if (this.snap) {
      this.pos.copy(p);
      this.look.copy(l);
      this.snap = false;
    } else {
      const k = 1 - Math.exp(-dt * (this.mode === 'chase' ? 8 : 6));
      this.pos.lerp(p, k);
      this.look.lerp(l, k);
    }
    cam.position.copy(this.pos);
    // overhead: keep "downfield" pointing up on screen
    if (this.mode === 'overhead') cam.up.set(this.flip ? -s : s, 0, 0);
    else cam.up.set(0, 1, 0);
    cam.lookAt(this.look);
    if (cam.fov !== fov) {
      cam.fov = fov;
      cam.updateProjectionMatrix();
    }
  }
}
