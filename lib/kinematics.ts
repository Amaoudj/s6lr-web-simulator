import type {
  BodyPose,
  FootTarget,
  GaitName,
  GaitParameters,
  JointAngles,
  JointMap,
  LegId,
  MotionCommand,
} from "./simulator-types";

export type Vec3 = [number, number, number];

export const LEG_IDS: LegId[] = [1, 2, 3, 4, 5, 6];

export const LEG_META: Record<
  LegId,
  { short: string; name: string; side: 1 | -1; pair: LegId; station: string }
> = {
  1: { short: "RF", name: "Right front", side: 1, pair: 2, station: "front" },
  2: { short: "LF", name: "Left front", side: -1, pair: 1, station: "front" },
  3: { short: "RM", name: "Right middle", side: 1, pair: 4, station: "middle" },
  4: { short: "LM", name: "Left middle", side: -1, pair: 3, station: "middle" },
  5: { short: "RR", name: "Right rear", side: 1, pair: 6, station: "rear" },
  6: { short: "LR", name: "Left rear", side: -1, pair: 5, station: "rear" },
};

export const HIP_CENTERS: Record<LegId, Vec3> = {
  1: [-0.05, 0.0245, -0.0415],
  2: [-0.05, -0.0245, -0.0415],
  3: [0, 0.028, -0.0415],
  4: [0, -0.028, -0.0415],
  5: [0.05, 0.0245, -0.0415],
  6: [0.05, -0.0245, -0.0415],
};

export const LINK_LENGTHS = {
  coxa: 0.024,
  femur: Math.hypot(0.0165, 0.054),
  tibia: Math.hypot(0.0045, 0.077),
} as const;

const FEMUR_ZERO = Math.atan2(0.054, 0.0165);
const TIBIA_DELTA_ZERO = Math.atan2(-0.077, 0.0045) - FEMUR_ZERO;
const NOMINAL_HEIGHT = 0.1;

export const JOINT_LIMITS = {
  hip: [-1, 1],
  femur: [-1.25, 1.25],
  tibia: [-1.25, 1.57],
} as const;

export const STRICT_XML_LIMITS = {
  hip: [-1, 1],
  femur: [-1, 1],
  tibia: [-1, 1],
} as const;

export const GAIT_LIBRARY: Record<
  GaitName,
  {
    label: string;
    code: string;
    period: number;
    duty: number;
    description: string;
    starts: Record<LegId, number>;
  }
> = {
  tripod: {
    label: "Tripod",
    code: "3+3",
    period: 0.8,
    duty: 0.5,
    description: "Fast alternating tripods",
    starts: { 1: 0, 2: 0.5, 3: 0.5, 4: 0, 5: 0, 6: 0.5 },
  },
  ripple: {
    label: "Ripple",
    code: "2×3",
    period: 1.2,
    duty: 2 / 3,
    description: "Diagonal pairs, medium stability",
    starts: { 1: 2 / 3, 2: 0, 3: 1 / 3, 4: 2 / 3, 5: 0, 6: 1 / 3 },
  },
  wave: {
    label: "Wave",
    code: "1×6",
    period: 1.8,
    duty: 5 / 6,
    description: "One leg swings at a time",
    starts: { 1: 2 / 6, 2: 5 / 6, 3: 1 / 6, 4: 4 / 6, 5: 0, 6: 3 / 6 },
  },
  tetrapod: {
    label: "Tetrapod",
    code: "2+2+2",
    period: 1.05,
    duty: 2 / 3,
    description: "Three alternating leg pairs",
    starts: { 1: 0, 2: 1 / 3, 3: 2 / 3, 4: 0, 5: 1 / 3, 6: 2 / 3 },
  },
};

export const STANDING_JOINTS: JointMap = {
  1: { hip: 0.75, femur: -1.15, tibia: 1.1 },
  2: { hip: -0.75, femur: 1.15, tibia: -1.1 },
  3: { hip: 0, femur: -1.15, tibia: 1 },
  4: { hip: 0, femur: 1.15, tibia: -1 },
  5: { hip: -0.75, femur: -1.15, tibia: 1.1 },
  6: { hip: 0.75, femur: 1.15, tibia: -1.1 },
};

export const cloneJointMap = (source: JointMap = STANDING_JOINTS): JointMap =>
  Object.fromEntries(
    LEG_IDS.map((id) => [id, { ...source[id] }]),
  ) as unknown as JointMap;

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const subtract = (a: Vec3, b: Vec3): Vec3 => [
  a[0] - b[0],
  a[1] - b[1],
  a[2] - b[2],
];
const scale = (v: Vec3, factor: number): Vec3 => [
  v[0] * factor,
  v[1] * factor,
  v[2] * factor,
];

const rotateX = (v: Vec3, angle: number): Vec3 => {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [v[0], c * v[1] - s * v[2], s * v[1] + c * v[2]];
};

const rotateY = (v: Vec3, angle: number): Vec3 => {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]];
};

const rotateZ = (v: Vec3, angle: number): Vec3 => {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
};

export const clamp = (value: number, low: number, high: number) =>
  Math.min(high, Math.max(low, value));

export const wrapAngle = (value: number) => {
  let result = value;
  while (result > Math.PI) result -= Math.PI * 2;
  while (result < -Math.PI) result += Math.PI * 2;
  return result;
};

export const degrees = (radians: number) => (radians * 180) / Math.PI;
export const radians = (degreesValue: number) => (degreesValue * Math.PI) / 180;

export function forwardKinematics(leg: LegId, q: JointAngles): Vec3 {
  const side = LEG_META[leg].side;
  const coxa: Vec3 = [0, side * 0.024, 0.0115];
  const femur: Vec3 = [0, side * 0.0165, 0.054];
  const tibia: Vec3 = [0, side * 0.0045, -0.077];
  const chain = add(
    coxa,
    add(rotateX(femur, q.femur), rotateX(tibia, q.femur + q.tibia)),
  );
  return add(HIP_CENTERS[leg], rotateZ(chain, q.hip));
}

export const STANDING_FEET = Object.fromEntries(
  LEG_IDS.map((id) => [id, forwardKinematics(id, STANDING_JOINTS[id])]),
) as unknown as Record<LegId, Vec3>;

export type IkResult = {
  angles: JointAngles;
  reachable: boolean;
  residual: number;
  target: Vec3;
  solved: Vec3;
};

export function inverseKinematics(leg: LegId, target: Vec3): IkResult {
  const side = LEG_META[leg].side;
  const d = subtract(target, HIP_CENTERS[leg]);
  const hip = wrapAngle(Math.atan2(d[1], d[0]) - side * Math.PI * 0.5);
  const rho = Math.hypot(d[0], d[1]);
  const r = rho - LINK_LENGTHS.coxa;
  const z = d[2] - 0.0115;
  const distance = Math.hypot(r, z);
  const minReach = Math.abs(LINK_LENGTHS.femur - LINK_LENGTHS.tibia);
  const maxReach = LINK_LENGTHS.femur + LINK_LENGTHS.tibia;
  const reachable =
    rho > LINK_LENGTHS.coxa &&
    distance >= minReach &&
    distance <= maxReach &&
    Math.abs(hip) <= JOINT_LIMITS.hip[1];
  const cosine = clamp(
    (r * r +
      z * z -
      LINK_LENGTHS.femur * LINK_LENGTHS.femur -
      LINK_LENGTHS.tibia * LINK_LENGTHS.tibia) /
      (2 * LINK_LENGTHS.femur * LINK_LENGTHS.tibia),
    -1,
    1,
  );
  const delta = -Math.acos(cosine);
  const a =
    Math.atan2(z, r) -
    Math.atan2(
      LINK_LENGTHS.tibia * Math.sin(delta),
      LINK_LENGTHS.femur + LINK_LENGTHS.tibia * Math.cos(delta),
    );
  const alpha = a - FEMUR_ZERO;
  const beta = delta - TIBIA_DELTA_ZERO;
  const angles: JointAngles = {
    hip: clamp(hip, JOINT_LIMITS.hip[0], JOINT_LIMITS.hip[1]),
    femur: clamp(side * alpha, JOINT_LIMITS.femur[0], JOINT_LIMITS.femur[1]),
    tibia: clamp(side * beta, JOINT_LIMITS.tibia[0], JOINT_LIMITS.tibia[1]),
  };
  const solved = forwardKinematics(leg, angles);
  const residual = Math.hypot(
    solved[0] - target[0],
    solved[1] - target[1],
    solved[2] - target[2],
  );
  return {
    angles,
    reachable:
      reachable &&
      residual < 0.004 &&
      Math.abs(angles.femur - side * alpha) < 1e-5 &&
      Math.abs(angles.tibia - side * beta) < 1e-5,
    residual,
    target,
    solved,
  };
}

export function footTargetForLeg(leg: LegId, target: FootTarget): Vec3 {
  return [target.x, LEG_META[leg].side * target.outward, target.z];
}

export function computeJacobian(leg: LegId, q: JointAngles) {
  const base = forwardKinematics(leg, q);
  const epsilon = 1e-5;
  const columns = (["hip", "femur", "tibia"] as const).map((joint) => {
    const next = { ...q, [joint]: q[joint] + epsilon };
    return scale(subtract(forwardKinematics(leg, next), base), 1 / epsilon);
  });
  return [
    [columns[0][0], columns[1][0], columns[2][0]],
    [columns[0][1], columns[1][1], columns[2][1]],
    [columns[0][2], columns[1][2], columns[2][2]],
  ];
}

export function determinant3(matrix: number[][]) {
  return (
    matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
    matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
    matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
  );
}

export function canonicalAngles(leg: LegId, raw: JointAngles): JointAngles {
  const side = LEG_META[leg].side;
  return {
    hip: raw.hip * side,
    femur: raw.femur * side,
    tibia: raw.tibia * side,
  };
}

export function rawAngles(leg: LegId, canonical: JointAngles): JointAngles {
  return canonicalAngles(leg, canonical);
}

function inverseBodyRotation(point: Vec3, pose: BodyPose): Vec3 {
  let result = rotateZ(point, -pose.yaw);
  result = rotateY(result, -pose.pitch);
  result = rotateX(result, -pose.roll);
  return result;
}

export function solveBodyPose(pose: BodyPose): JointMap {
  const result = cloneJointMap();
  for (const leg of LEG_IDS) {
    const base = STANDING_FEET[leg];
    const worldRelative: Vec3 = [
      base[0],
      base[1],
      base[2] - (pose.height - NOMINAL_HEIGHT),
    ];
    result[leg] = inverseKinematics(leg, inverseBodyRotation(worldRelative, pose)).angles;
  }
  return result;
}

const smoothstep = (value: number) => value * value * (3 - 2 * value);
const fract = (value: number) => ((value % 1) + 1) % 1;

export function solveGaitPose(
  gait: GaitName,
  globalPhase: number,
  parameters: GaitParameters,
  command: MotionCommand,
  bodyPose: BodyPose,
) {
  const config = GAIT_LIBRARY[gait];
  const joints = cloneJointMap();
  const contacts = {} as Record<LegId, boolean>;
  const feet = {} as Record<LegId, Vec3>;
  const reachability = {} as Record<LegId, boolean>;
  const swingFraction = 1 - parameters.duty;

  for (const leg of LEG_IDS) {
    const base = STANDING_FEET[leg];
    const turnAngle = command.turn * (parameters.stride / 0.1);
    const displacement: Vec3 = [
      -command.forward * parameters.stride - base[1] * turnAngle,
      command.lateral * parameters.stride + base[0] * turnAngle,
      0,
    ];
    const u = fract(globalPhase - config.starts[leg]);
    let localTarget: Vec3;

    if (u < swingFraction) {
      const tau = swingFraction > 0 ? u / swingFraction : 0;
      const eased = smoothstep(tau);
      localTarget = add(
        base,
        add(
          scale(displacement, -0.5 + eased),
          [0, 0, parameters.lift * Math.sin(Math.PI * tau)],
        ),
      );
      contacts[leg] = false;
    } else {
      const tau = parameters.duty > 0 ? (u - swingFraction) / parameters.duty : 0;
      localTarget = add(base, scale(displacement, 0.5 - tau));
      contacts[leg] = true;
    }

    localTarget[2] -= bodyPose.height - NOMINAL_HEIGHT;
    localTarget = inverseBodyRotation(localTarget, bodyPose);
    const solution = inverseKinematics(leg, localTarget);
    joints[leg] = solution.angles;
    feet[leg] = solution.solved;
    reachability[leg] = solution.reachable;
  }

  return { joints, contacts, feet, reachability };
}

export function gaitCyclePeriod(gait: GaitName, speed: number) {
  return GAIT_LIBRARY[gait].period / Math.max(0.1, speed);
}

export function jointActivity(q: JointAngles) {
  return Math.min(
    1,
    (Math.abs(q.hip) / 1 +
      Math.abs(q.femur) / 1.25 +
      Math.abs(q.tibia) / 1.57) /
      3,
  );
}
