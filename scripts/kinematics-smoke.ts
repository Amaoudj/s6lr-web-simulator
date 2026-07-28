import {
  GAIT_LIBRARY,
  LEG_IDS,
  STANDING_JOINTS,
  forwardKinematics,
  inverseKinematics,
  solveGaitPose,
} from "../lib/kinematics.ts";
import type { GaitName } from "../lib/simulator-types.ts";

let maxRoundTripError = 0;
for (const leg of LEG_IDS) {
  const foot = forwardKinematics(leg, STANDING_JOINTS[leg]);
  const solution = inverseKinematics(leg, foot);
  const expected = STANDING_JOINTS[leg];
  maxRoundTripError = Math.max(
    maxRoundTripError,
    Math.abs(expected.hip - solution.angles.hip),
    Math.abs(expected.femur - solution.angles.femur),
    Math.abs(expected.tibia - solution.angles.tibia),
  );
  if (!solution.reachable || solution.residual > 1e-10) {
    throw new Error(
      `Standing FK/IK round trip failed for leg ${leg}: ${solution.residual} m`,
    );
  }
}

let unreachableSamples = 0;
for (const gait of Object.keys(GAIT_LIBRARY) as GaitName[]) {
  for (let index = 0; index < 120; index += 1) {
    const pose = solveGaitPose(
      gait,
      index / 120,
      {
        speed: 1,
        stride: 0.025,
        lift: 0.012,
        duty: GAIT_LIBRARY[gait].duty,
      },
      { forward: 0.75, lateral: 0, turn: 0 },
      { height: 0.1, roll: 0, pitch: 0, yaw: 0 },
    );
    for (const leg of LEG_IDS) {
      if (!pose.reachability[leg]) unreachableSamples += 1;
    }
  }
}

console.log(
  JSON.stringify(
    {
      standingRoundTripRadians: maxRoundTripError,
      sampledGaits: Object.keys(GAIT_LIBRARY),
      gaitSamples: 120 * Object.keys(GAIT_LIBRARY).length * LEG_IDS.length,
      unreachableSamples,
    },
    null,
    2,
  ),
);

if (maxRoundTripError > 1e-10 || unreachableSamples > 0) process.exit(1);
