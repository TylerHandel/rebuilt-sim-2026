// Neural-network driver: drives a robot (AUTO and TELEOP) with a trained Policy, deciding
// 10 times a second like the training workers do.
import { buildObs, actionToCmd, DECISION_DT, OBS_DIM } from './obs.js';

export function nearest(robot, list) {
  let best = null, bd = Infinity;
  for (const r of list) {
    const d = (r.pos.x - robot.pos.x) ** 2 + (r.pos.z - robot.pos.z) ** 2;
    if (d < bd) { bd = d; best = r; }
  }
  return best;
}

export class NNDriver {
  // foes: the other ALLIANCE's robots; the network watches the nearest one
  // foes / mates: the robots on the other ALLIANCE and its own
  constructor({ robot, foes = [], mates = [], match, fuel, policy, sampleButtons = true }) {
    Object.assign(this, { robot, foes, mates, match, fuel, policy, sampleButtons });
    this.drivesAuto = true; // the network plays the whole match, AUTO included
    this.t = DECISION_DT;   // decide on the first step
    this.obs = new Float32Array(OBS_DIM);
    this.strategy = 'nn';
    this.label = 'Neural net';
    this.status = 'Neural net';
  }

  update(dt) {
    this.t += dt;
    if (this.t < DECISION_DT - 1e-9) return;
    this.t = 0;
    buildObs(this.robot, { foes: this.foes, mates: this.mates }, this.match, this.fuel, this.obs);
    const { action } = this.policy.act(this.obs, { sampleButtons: this.sampleButtons });
    actionToCmd(this.robot, action, this.robot.cmd);
  }
}
