// Strategy learned by AI training. Written by tools/train.mjs or exported from the in-game
// AI Tuning screen; replace this file to change the shipped "Trained" AI.
export const TRAINED_BRAIN = {
  fill: 0.85, // fraction of the hopper (up to 60 FUEL) collected before a cycle
  cycleTime: 14, // s of collecting before it scores what it has while its HUB is active
  stageMargin: 2, // s of slack when heading in to stage before its HUB turns active
  topUp: 6, // tops up its hopper if its HUB stays inactive this much longer than the trip back (s)
  spotFx: 2.7, // shooting spot distance from its ALLIANCE WALL (m)
  spotZ: 0.9, // minimum sideways offset of the shooting spot from the HUB (m)
  pushThrough: 1.5, // s of being blocked on the way to its zone before it stops going around and drives through
  shuttle: 1, // during an inactive shift, keep collecting and pass FUEL into its ALLIANCE ZONE instead of waiting there (on at 50% or more)
  shuttleKeep: 0.75, // fraction of its hopper it keeps while shuttling; it passes the rest
  collectSpeed: 0.7, // speed among FUEL (fraction of top speed)
  density: 0.35, // preference for dense FUEL clusters over the nearest FUEL
  ownZone: 0.4, // preference for FUEL in its own ALLIANCE ZONE (m)
  intakeDist: 2.5, // distance from the target FUEL at which the intake drops (m)
  stockpile: 1.5, // extra preference for FUEL in its own ALLIANCE ZONE while its HUB is active (m)
  zoneMin: 6, // if fewer FUEL than this are lying in its own ALLIANCE ZONE it ignores them and goes to the NEUTRAL ZONE
  pinLimit: 1.8, // s it holds a PIN before backing off
  pushSpeed: 1, // speed it hits the other robot at (fraction of top speed)
  ramDist: 2.5, // charges the other robot once it comes this close while guarding the lane to its zone (m)
  blockLead: 1.2, // how far ahead of the other robot, toward the lane into its zone, it guards (m)
  engage: 0, // starts shoving once the other robot is this close to its ALLIANCE ZONE (m)
  hybridLoad: 0, // Hybrid only goes to defend once its hopper is at least this full
};

export const TRAINING_INFO = {
  "source": "hand-tuned defaults (no training run saved yet)"
};
