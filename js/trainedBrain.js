// Strategy learned by AI training. Written by tools/train.mjs or exported from the in-game
// AI Tuning screen; replace this file to change the shipped "Trained" AI.
export const TRAINED_BRAIN = {
  fill: 0.799, // fraction of the hopper (up to 60 FUEL) collected before a cycle
  cycleTime: 16.908, // s of collecting before it scores what it has while its HUB is active
  stageMargin: 2.334, // s of slack when heading in to stage before its HUB turns active
  topUp: 4.434, // tops up its hopper if its HUB stays inactive this much longer than the trip back (s)
  spotFx: 2.919, // shooting spot distance from its ALLIANCE WALL for a chassis-aimed shooter (turrets shoot from anywhere in the zone) (m)
  spotZ: 1.11, // minimum sideways offset of the chassis-aimed shooting spot from the HUB (m)
  shootSpeed: 0.367, // top speed while shooting or passing on the move, so the turret/flywheel can settle (fraction of top speed)
  shootAccel: 3.447, // how quickly it changes velocity while shooting or passing (m/s²)
  pushThrough: 1.99, // s of being blocked on the way to its zone before it stops going around and drives through
  shuttle: 0.97, // during an inactive shift, collect in the NEUTRAL ZONE and pass everything into its ALLIANCE ZONE, then load up just before its HUB turns on (on at 50% or more)
  passBatch: 12.665, // a chassis-aimed shooter passes once it holds this many FUEL (turrets pass while they collect)
  dropFx: 2.557, // distance from its ALLIANCE WALL where its passes land (m)
  collectSpeed: 0.778, // speed among FUEL (fraction of top speed)
  density: 0.356, // preference for dense FUEL clusters over the nearest FUEL
  ownZone: -0.121, // preference for FUEL in its own ALLIANCE ZONE (m)
  intakeDist: 2.937, // distance from the target FUEL at which the intake drops (m)
  stockpile: 1.859, // extra preference for FUEL in its own ALLIANCE ZONE while its HUB is active (m)
  steal: 2.208, // extra preference for FUEL in the opponent's ALLIANCE ZONE: taking it both denies them and feeds us (m)
  zoneMin: 6.747, // if fewer FUEL than this are lying in its own ALLIANCE ZONE it ignores them and goes to the NEUTRAL ZONE
  pinLimit: 1.63, // s it holds a PIN before backing off
  pushSpeed: 0.935, // speed it hits the other robot at (fraction of top speed)
  ramDist: 2.262, // charges the other robot once it comes this close while guarding the lane to its zone (m)
  blockLead: 1.266, // how far ahead of the other robot, toward the lane into its zone, it guards (m)
  engage: -0.353, // starts shoving once the other robot is this close to its ALLIANCE ZONE (m)
  hybridLoad: 0.101, // Hybrid only goes to defend once its hopper is at least this full
};

export const TRAINING_INFO = {
  "source": "tools/train.mjs self-play",
  "from": "hand-tuned brain",
  "date": "2026-09-25",
  "generations": 8,
  "population": 8,
  "scenariosPerGen": 5,
  "matches": 384,
  "minutes": 57,
  "seed": 2028,
  "resumed": false,
  "validation": {
    "scenarios": 12,
    "avgGain": 10,
    "better": 7,
    "worse": 5,
    "byStrategy": {
      "scorer": 25,
      "hybrid": 1,
      "defense": -5
    },
    "trainedFoulPts": 10,
    "handTunedFoulPts": 20
  }
};
