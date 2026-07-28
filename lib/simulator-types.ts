export type GaitName = "tripod" | "ripple" | "wave" | "tetrapod";
export type ControlMode = "gait" | "joints" | "ik" | "body";
export type ViewPreset = "perspective" | "top" | "front" | "side";
export type LegId = 1 | 2 | 3 | 4 | 5 | 6;

export type JointAngles = {
  hip: number;
  femur: number;
  tibia: number;
};

export type JointMap = Record<LegId, JointAngles>;

export type BodyPose = {
  height: number;
  roll: number;
  pitch: number;
  yaw: number;
};

export type MotionCommand = {
  forward: number;
  lateral: number;
  turn: number;
};

export type GaitParameters = {
  speed: number;
  stride: number;
  lift: number;
  duty: number;
};

export type FootTarget = {
  x: number;
  outward: number;
  z: number;
};

export type RobotStatus = {
  loaded: boolean;
  loadingProgress: number;
  error: string | null;
  fps: number;
  triangleCount: number;
};

export type FrameTelemetry = {
  joints: JointMap;
  contacts: Record<LegId, boolean>;
  reachability: Record<LegId, boolean>;
  phase: number;
  cycle: number;
  fps: number;
};

export type ViewportHandle = {
  setView: (view: ViewPreset) => void;
  resetCamera: () => void;
};
