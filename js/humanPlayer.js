// HUMAN PLAYER at the OUTPOST. FUEL may only enter the FIELD through the CHUTE, the OUTPOST
// base opening, or be thrown from the OUTPOST AREA (G425). Off-field FUEL storage is limited to
// the CHUTE and CORRAL (G427). The HUMAN PLAYER may also throw FUEL straight into their HUB.
import { HUB, OUTPOST, FUEL } from './constants.js';
import { Field } from './field.js';
import { ShotTable, solveMovingShot } from './ballistics.js';
import { gauss, rand, DEG } from './util.js';

export class HumanPlayer {
  constructor({ alliance, field, fuel, match }) {
    this.alliance = alliance;
    this.field = field;
    this.fuel = fuel;
    this.match = match;
    this.info = field.outposts[alliance];
    const h0 = this.info.hpThrowPos.y;
    this.table = new ShotTable({
      h0, Ht: HUB.targetHeight, hoodMin: 18, hoodMax: 62, speedMax: 14, mode: 'hub',
      clearDist: HUB.size / 2 + FUEL.radius + 0.02, clearHeight: HUB.rimFront + FUEL.radius + 0.04,
    });
    this.auto = false;
    this.reset();
  }

  reset() {
    this.doorOpen = false;
    this.releaseTimer = 0;
    this.throwCooldown = 0;
    this.reloadTimer = 0;
    this.armT = 0;
    this.thrown = 0;
    this.released = 0;
  }

  toggleDoor() {
    this.doorOpen = !this.doorOpen;
  }

  // Throw one FUEL at the HUB (from the CORRAL first, otherwise from the CHUTE)
  throwOne(t) {
    if (this.throwCooldown > 0) return false;
    let b = this.fuel.corralBalls(this.alliance)[0];
    if (b) this.fuel.toRobot(b); // picked up by the HUMAN PLAYER
    else b = this.fuel.takeFromChute(this.alliance);
    if (!b) return false;
    const p = this.info.hpThrowPos;
    const c = Field.hubCenter(this.alliance);
    const sol = solveMovingShot(this.table, p, { x: 0, z: 0 }, c);
    if (!sol) {
      this.fuel.addToChute(this.alliance, b);
      return false;
    }
    // a good HUMAN PLAYER lands roughly half of these ~6m throws
    const psi = sol.psi + gauss() * 2.8 * DEG;
    const th = sol.theta + gauss() * 3.0 * DEG;
    const v = sol.v * (1 + gauss() * 0.045);
    this.fuel.launch(b, { x: p.x, y: p.y, z: p.z }, {
      x: Math.cos(psi) * Math.cos(th) * v,
      y: Math.sin(th) * v,
      z: -Math.sin(psi) * Math.cos(th) * v,
    }, { by: 'hp', alliance: this.alliance, legal: true, t, fromOutside: true });
    this.throwCooldown = 1.05;
    this.armT = 0.35;
    this.thrown++;
    return true;
  }

  update(dt, t, controls) {
    this.throwCooldown = Math.max(0, this.throwCooldown - dt);
    const live = this.match.phase === 'auto' || this.match.phase === 'teleop';
    if (live) {
      if (controls.toggleDoor) this.toggleDoor();
      const wantThrow = controls.throwHeld || (this.auto && this.match.hubActive(this.alliance));
      if (wantThrow) this.throwOne(t);
      // CHUTE DOOR open: FUEL rolls down the 15deg CHUTE and out the upper opening
      if (this.doorOpen) {
        this.releaseTimer += dt;
        while (this.releaseTimer >= 0.15) {
          this.releaseTimer -= 0.15;
          const b = this.fuel.takeFromChute(this.alliance);
          if (!b) break;
          const o = this.info.upperOpening;
          const s = this.info.intoField;
          this.fuel.launch(b, { x: o.x, y: o.y, z: o.z + rand(-o.halfW, o.halfW) }, { x: s * rand(1.3, 2.0), y: rand(-0.3, 0.1), z: rand(-0.25, 0.25) }, { by: 'hp', alliance: this.alliance, legal: true, t });
          this.released++;
        }
      } else this.releaseTimer = 0;
      // reload the CHUTE from the CORRAL when the HUMAN PLAYER isn't throwing
      this.reloadTimer += dt;
      if (!wantThrow && this.reloadTimer > 0.7 && this.fuel.chuteCount(this.alliance) < OUTPOST.chuteCapacity) {
        const b = this.fuel.corralBalls(this.alliance)[0];
        if (b) {
          this.fuel.addToChute(this.alliance, b);
          this.reloadTimer = 0;
          this.armT = 0.3;
        }
      }
    }
    // visuals
    this.armT = Math.max(0, this.armT - dt);
    const arm = this.info.hpArm;
    arm.rotation.z = -0.6 + (this.armT > 0 ? Math.sin((this.armT / 0.35) * Math.PI) * 1.8 : 0);
    this.info.door.rotation.z = this.doorOpen ? -1.4 : 0;
    this.info.door.position.y = this.doorOpen ? OUTPOST.upperH + 0.04 : OUTPOST.upperH / 2;
  }

  corralCount() {
    return this.fuel.corralBalls(this.alliance).length;
  }
}
