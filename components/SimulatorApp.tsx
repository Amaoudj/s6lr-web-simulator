"use client";

import {
  Activity,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  ArrowUp,
  Box,
  Braces,
  Camera,
  Check,
  ChevronDown,
  CircleGauge,
  Crosshair,
  Download,
  Eye,
  Footprints,
  Gauge,
  Grid3X3,
  Hexagon,
  Info,
  Maximize2,
  Move3d,
  Orbit,
  Pause,
  Play,
  RotateCcw,
  RotateCw,
  SlidersHorizontal,
  Square,
  StepForward,
  Upload,
  Waves,
  Zap,
} from "lucide-react";
import {
  type ChangeEvent,
  type ComponentType,
  type CSSProperties,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import {
  canonicalAngles,
  clamp,
  cloneJointMap,
  computeJacobian,
  degrees,
  determinant3,
  footTargetForLeg,
  forwardKinematics,
  GAIT_LIBRARY,
  inverseKinematics,
  jointActivity,
  JOINT_LIMITS,
  LEG_IDS,
  LEG_META,
  radians,
  solveBodyPose,
  STANDING_FEET,
  STANDING_JOINTS,
} from "../lib/kinematics";
import type {
  BodyPose,
  ControlMode,
  FootTarget,
  FrameTelemetry,
  GaitName,
  GaitParameters,
  JointAngles,
  JointMap,
  LegId,
  MotionCommand,
  RobotStatus,
  ViewportHandle,
  ViewPreset,
} from "../lib/simulator-types";
import { RobotViewport } from "./RobotViewport";

type IconType = ComponentType<{ size?: number; strokeWidth?: number }>;

const initialContacts = Object.fromEntries(
  LEG_IDS.map((leg) => [leg, true]),
) as Record<LegId, boolean>;
const initialReachability = { ...initialContacts };

const INITIAL_BODY: BodyPose = {
  height: 0.1,
  roll: 0,
  pitch: 0,
  yaw: 0,
};

const INITIAL_GAIT: GaitParameters = {
  speed: 1,
  stride: 0.025,
  lift: 0.012,
  duty: GAIT_LIBRARY.tripod.duty,
};

const INITIAL_MOTION: MotionCommand = {
  forward: 0.75,
  lateral: 0,
  turn: 0,
};

function RangeControl({
  label,
  value,
  min,
  max,
  step,
  unit,
  format,
  onChange,
  disabled,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  unit?: string;
  format?: (value: number) => string;
  onChange: (value: number) => void;
  disabled?: boolean;
}) {
  const ratio = ((value - min) / (max - min)) * 100;
  return (
    <label className={`range-control${disabled ? " is-disabled" : ""}`}>
      <span className="range-meta">
        <span>{label}</span>
        <output>
          {format ? format(value) : value.toFixed(2)}
          {unit && <small>{unit}</small>}
        </output>
      </span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        disabled={disabled}
        style={{ "--range-progress": `${ratio}%` } as CSSProperties}
        onChange={(event) => onChange(Number(event.target.value))}
        aria-label={label}
      />
    </label>
  );
}

function Sparkline({
  phase,
  amplitude,
  secondary,
}: {
  phase: number;
  amplitude: number;
  secondary?: boolean;
}) {
  const points = Array.from({ length: 42 }, (_, index) => {
    const t = index / 41;
    const wave =
      Math.sin((t * 2 + phase) * Math.PI * 2) * 0.55 +
      Math.sin((t * 5.3 + phase * 0.7) * Math.PI * 2) * 0.16;
    const x = t * 120;
    const y = 19 - wave * amplitude * 13;
    return `${x.toFixed(2)},${y.toFixed(2)}`;
  }).join(" ");
  return (
    <svg className="sparkline" viewBox="0 0 120 38" preserveAspectRatio="none" aria-hidden="true">
      <line x1="0" y1="19" x2="120" y2="19" />
      <polyline className={secondary ? "secondary" : ""} points={points} />
    </svg>
  );
}

function MetricDial({
  value,
  label,
  unit,
  tone = "orange",
}: {
  value: number;
  label: string;
  unit: string;
  tone?: "orange" | "green";
}) {
  const normalized = clamp(value, 0, 100);
  return (
    <div
      className={`metric-dial ${tone}`}
      style={
        {
          "--dial-value": `${normalized * 3.6}deg`,
        } as CSSProperties
      }
    >
      <div className="dial-core">
        <strong>{Math.round(value)}</strong>
        <small>{unit}</small>
      </div>
      <span>{label}</span>
    </div>
  );
}

function SectionHeader({
  eyebrow,
  title,
  action,
}: {
  eyebrow: string;
  title: string;
  action?: React.ReactNode;
}) {
  return (
    <div className="section-heading">
      <div>
        <span>{eyebrow}</span>
        <h2>{title}</h2>
      </div>
      {action}
    </div>
  );
}

function IconButton({
  icon: Icon,
  label,
  active,
  onClick,
}: {
  icon: IconType;
  label: string;
  active?: boolean;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      className={`icon-button${active ? " is-active" : ""}`}
      onClick={onClick}
      title={label}
      aria-label={label}
    >
      <Icon size={16} strokeWidth={1.7} />
    </button>
  );
}

export function SimulatorApp() {
  const viewportRef = useRef<ViewportHandle>(null);
  const importInputRef = useRef<HTMLInputElement>(null);
  const [mode, setMode] = useState<ControlMode>("gait");
  const [gait, setGait] = useState<GaitName>("tripod");
  const [gaitParameters, setGaitParameters] =
    useState<GaitParameters>(INITIAL_GAIT);
  const [motion, setMotion] = useState<MotionCommand>(INITIAL_MOTION);
  const [bodyPose, setBodyPose] = useState<BodyPose>(INITIAL_BODY);
  const [jointTargets, setJointTargets] = useState<JointMap>(() =>
    cloneJointMap(),
  );
  const [playing, setPlaying] = useState(false);
  const [selectedLeg, setSelectedLeg] = useState<LegId>(1);
  const [mirror, setMirror] = useState(true);
  const [showGrid, setShowGrid] = useState(true);
  const [showAxes, setShowAxes] = useState(false);
  const [resetToken, setResetToken] = useState(0);
  const [stepToken, setStepToken] = useState(0);
  const [robotStatus, setRobotStatus] = useState<RobotStatus>({
    loaded: false,
    loadingProgress: 0,
    error: null,
    fps: 0,
    triangleCount: 0,
  });
  const [telemetry, setTelemetry] = useState<FrameTelemetry>({
    joints: cloneJointMap(),
    contacts: initialContacts,
    reachability: initialReachability,
    phase: 0,
    cycle: 0,
    fps: 60,
  });
  const initialFoot = STANDING_FEET[1];
  const [ikTarget, setIkTarget] = useState<FootTarget>({
    x: initialFoot[0],
    outward: Math.abs(initialFoot[1]),
    z: initialFoot[2],
  });
  const [ikResidual, setIkResidual] = useState(0);
  const [ikReachable, setIkReachable] = useState(true);
  const [toast, setToast] = useState<string | null>(null);

  const selectedAngles = telemetry.joints[selectedLeg] ?? jointTargets[selectedLeg];
  const selectedFoot = useMemo(
    () => forwardKinematics(selectedLeg, selectedAngles),
    [selectedAngles, selectedLeg],
  );
  const selectedCanonical = useMemo(
    () => canonicalAngles(selectedLeg, selectedAngles),
    [selectedAngles, selectedLeg],
  );
  const jacobian = useMemo(
    () => computeJacobian(selectedLeg, selectedAngles),
    [selectedAngles, selectedLeg],
  );
  const manipulability = Math.abs(determinant3(jacobian)) * 1_000_000;
  const activeActivity =
    LEG_IDS.reduce(
      (sum, leg) => sum + jointActivity(telemetry.joints[leg]),
      0,
    ) / LEG_IDS.length;
  const contactCount = LEG_IDS.filter((leg) => telemetry.contacts[leg]).length;
  const supportScore = Math.round((contactCount / 6) * 100);

  const handleStatus = useCallback((status: RobotStatus) => {
    setRobotStatus((previous) => ({
      ...status,
      fps: status.fps || previous.fps,
    }));
  }, []);

  const handleTelemetry = useCallback((next: FrameTelemetry) => {
    setTelemetry(next);
    setRobotStatus((previous) => ({ ...previous, fps: next.fps }));
  }, []);

  const chooseMode = (nextMode: ControlMode) => {
    setPlaying(false);
    setJointTargets(cloneJointMap(telemetry.joints));
    setMode(nextMode);
  };

  const chooseGait = (nextGait: GaitName) => {
    setGait(nextGait);
    setGaitParameters((current) => ({
      ...current,
      duty: GAIT_LIBRARY[nextGait].duty,
    }));
  };

  const resetSimulator = useCallback(() => {
    setPlaying(false);
    setGait("tripod");
    setGaitParameters(INITIAL_GAIT);
    setMotion(INITIAL_MOTION);
    setBodyPose(INITIAL_BODY);
    setJointTargets(cloneJointMap());
    setIkTarget({
      x: STANDING_FEET[selectedLeg][0],
      outward: Math.abs(STANDING_FEET[selectedLeg][1]),
      z: STANDING_FEET[selectedLeg][2],
    });
    setIkResidual(0);
    setIkReachable(true);
    setResetToken((token) => token + 1);
    setToast("Simulator reset to calibrated stance");
  }, [selectedLeg]);

  const updateBodyPose = (
    key: keyof BodyPose,
    value: number,
    solve = true,
  ) => {
    const next = { ...bodyPose, [key]: value };
    setBodyPose(next);
    if (solve) setJointTargets(solveBodyPose(next));
  };

  const updateJoint = (joint: keyof JointAngles, value: number) => {
    setPlaying(false);
    setJointTargets((current) => {
      const next = cloneJointMap(current);
      next[selectedLeg][joint] = value;
      if (mirror) {
        const pair = LEG_META[selectedLeg].pair;
        next[pair][joint] = -value;
      }
      return next;
    });
  };

  const selectLeg = (leg: LegId) => {
    setSelectedLeg(leg);
    const foot = forwardKinematics(leg, telemetry.joints[leg] ?? jointTargets[leg]);
    setIkTarget({ x: foot[0], outward: Math.abs(foot[1]), z: foot[2] });
  };

  const solveIkTarget = (target: FootTarget, updateTarget = true) => {
    if (updateTarget) setIkTarget(target);
    const solution = inverseKinematics(
      selectedLeg,
      footTargetForLeg(selectedLeg, target),
    );
    setIkReachable(solution.reachable);
    setIkResidual(solution.residual);
    setPlaying(false);
    setJointTargets((current) => {
      const next = cloneJointMap(current);
      next[selectedLeg] = solution.angles;
      if (mirror) {
        const pair = LEG_META[selectedLeg].pair;
        next[pair] = {
          hip: -solution.angles.hip,
          femur: -solution.angles.femur,
          tibia: -solution.angles.tibia,
        };
      }
      return next;
    });
  };

  const setDirection = (next: Partial<MotionCommand>) => {
    setMotion((current) => ({ ...current, ...next }));
  };

  useEffect(() => {
    if (!toast) return;
    const timer = window.setTimeout(() => setToast(null), 2600);
    return () => window.clearTimeout(timer);
  }, [toast]);

  useEffect(() => {
    const held = new Set<string>();
    const updateFromKeys = () => {
      const forward = (held.has("w") || held.has("arrowup") ? 1 : 0) -
        (held.has("s") || held.has("arrowdown") ? 1 : 0);
      const lateral = (held.has("d") || held.has("arrowright") ? 1 : 0) -
        (held.has("a") || held.has("arrowleft") ? 1 : 0);
      const turn = (held.has("e") ? 1 : 0) - (held.has("q") ? 1 : 0);
      if (held.size > 0 || forward || lateral || turn) {
        setMotion({ forward, lateral, turn });
      }
    };
    const isTyping = (target: EventTarget | null) =>
      target instanceof HTMLInputElement ||
      target instanceof HTMLTextAreaElement ||
      target instanceof HTMLSelectElement;
    const onKeyDown = (event: KeyboardEvent) => {
      if (isTyping(event.target)) return;
      const key = event.key.toLowerCase();
      if (key === " ") {
        event.preventDefault();
        setPlaying((current) => !current);
        return;
      }
      if (key === "r") {
        resetSimulator();
        return;
      }
      if (
        ["w", "a", "s", "d", "q", "e", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(
          key,
        )
      ) {
        event.preventDefault();
        held.add(key);
        updateFromKeys();
      }
    };
    const onKeyUp = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (!held.has(key)) return;
      held.delete(key);
      const forward = (held.has("w") || held.has("arrowup") ? 1 : 0) -
        (held.has("s") || held.has("arrowdown") ? 1 : 0);
      const lateral = (held.has("d") || held.has("arrowright") ? 1 : 0) -
        (held.has("a") || held.has("arrowleft") ? 1 : 0);
      const turn = (held.has("e") ? 1 : 0) - (held.has("q") ? 1 : 0);
      setMotion({ forward, lateral, turn });
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, [resetSimulator]);

  const exportPose = () => {
    const payload = {
      format: "s6lr-kinematics-pose",
      version: 1,
      exportedAt: new Date().toISOString(),
      model: "S6LR",
      source: "S6LR.xml / STL scale 0.001",
      gait,
      gaitParameters,
      bodyPose,
      motion,
      joints: telemetry.joints,
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `s6lr-pose-${Date.now()}.json`;
    anchor.click();
    URL.revokeObjectURL(url);
    setToast("Pose exported as JSON");
  };

  const importPose = async (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;
    try {
      const payload = JSON.parse(await file.text()) as {
        format?: string;
        gait?: GaitName;
        gaitParameters?: GaitParameters;
        bodyPose?: BodyPose;
        motion?: MotionCommand;
        joints?: JointMap;
      };
      if (payload.format !== "s6lr-kinematics-pose" || !payload.joints) {
        throw new Error("Not an S6LR pose file");
      }
      setPlaying(false);
      setJointTargets(cloneJointMap(payload.joints));
      if (payload.gait && GAIT_LIBRARY[payload.gait]) setGait(payload.gait);
      if (payload.gaitParameters) setGaitParameters(payload.gaitParameters);
      if (payload.bodyPose) setBodyPose(payload.bodyPose);
      if (payload.motion) setMotion(payload.motion);
      setToast(`Imported ${file.name}`);
    } catch (error) {
      setToast(error instanceof Error ? error.message : "Could not import pose");
    } finally {
      event.target.value = "";
    }
  };

  const tabs: Array<{
    id: ControlMode;
    label: string;
    sub: string;
    icon: IconType;
  }> = [
    { id: "gait", label: "Gaits", sub: "Locomotion", icon: Footprints },
    { id: "joints", label: "Joints", sub: "18-DOF", icon: SlidersHorizontal },
    { id: "ik", label: "Kinematics", sub: "FK / IK", icon: Crosshair },
    { id: "body", label: "Body pose", sub: "6-axis", icon: Move3d },
  ];

  const formatTriangles =
    robotStatus.triangleCount > 999_999
      ? `${(robotStatus.triangleCount / 1_000_000).toFixed(2)}M`
      : `${Math.round(robotStatus.triangleCount / 1000)}K`;

  return (
    <main className="sim-shell">
      <header className="topbar">
        <div className="brand-lockup">
          <div className="brand-mark" aria-hidden="true">
            <Hexagon size={23} strokeWidth={1.4} />
            <span>6</span>
          </div>
          <div>
            <span>SIX-LEGGED ROBOTICS</span>
            <h1>S6LR <b>//</b> KINEMATICS LAB</h1>
          </div>
        </div>
        <div className="system-strip">
          <div className="system-stat">
            <span>MODEL</span>
            <strong>S6LR · 18 DOF</strong>
          </div>
          <div className="system-stat">
            <span>RENDER</span>
            <strong>{robotStatus.loaded ? `${Math.round(robotStatus.fps)} FPS` : "LOADING"}</strong>
          </div>
          <div className="system-stat wide">
            <span>STATUS</span>
            <strong className={robotStatus.error ? "status-error" : "status-online"}>
              <i />
              {robotStatus.error ? "DEGRADED" : robotStatus.loaded ? "ONLINE" : "INITIALIZING"}
            </strong>
          </div>
          <div className="header-actions">
            <IconButton icon={Download} label="Export pose" onClick={exportPose} />
            <IconButton
              icon={Upload}
              label="Import pose"
              onClick={() => importInputRef.current?.click()}
            />
            <input
              ref={importInputRef}
              className="visually-hidden"
              type="file"
              accept="application/json,.json"
              onChange={importPose}
              aria-label="Import S6LR pose JSON"
            />
          </div>
        </div>
      </header>

      <nav className="mode-tabs" aria-label="Simulator control mode">
        {tabs.map(({ id, label, sub, icon: Icon }) => (
          <button
            key={id}
            type="button"
            className={mode === id ? "is-active" : ""}
            onClick={() => chooseMode(id)}
          >
            <Icon size={15} strokeWidth={1.6} />
            <span>
              <b>{label}</b>
              <small>{sub}</small>
            </span>
          </button>
        ))}
        <div className="model-provenance">
          <Box size={14} />
          <span>CAD SOURCE</span>
          <b>18 STL · 0.001 M/MM</b>
        </div>
      </nav>

      <div className="workbench">
        <aside className="telemetry-panel panel">
          <SectionHeader
            eyebrow="Live channels"
            title="Telemetry"
            action={<span className="sample-rate">8 Hz UI</span>}
          />

          <section className="telemetry-block trace-block">
            <div className="block-label">
              <span>JOINT ACTIVITY <small>EST.</small></span>
              <strong>{(activeActivity * 100).toFixed(0)}%</strong>
            </div>
            <Sparkline phase={telemetry.phase} amplitude={activeActivity + 0.28} />
            <div className="trace-legend">
              <span><i className="orange" /> commanded</span>
              <span>cycle {telemetry.cycle.toString().padStart(3, "0")}</span>
            </div>
          </section>

          <section className="telemetry-block trace-block">
            <div className="block-label">
              <span>BODY HEIGHT</span>
              <strong>{(bodyPose.height * 1000).toFixed(0)} <small>mm</small></strong>
            </div>
            <Sparkline phase={telemetry.phase * 0.25} amplitude={0.3} secondary />
            <div className="trace-legend">
              <span><i className="gray" /> kinematic target</span>
              <span>±0.2 mm</span>
            </div>
          </section>

          <section className="telemetry-block">
            <div className="block-label">
              <span>LEG CONTACT</span>
              <strong>{contactCount} / 6</strong>
            </div>
            <div className="contact-map">
              {LEG_IDS.map((leg) => (
                <button
                  type="button"
                  key={leg}
                  className={`${telemetry.contacts[leg] ? "contact" : "swing"}${
                    selectedLeg === leg ? " selected" : ""
                  }`}
                  onClick={() => selectLeg(leg)}
                  title={`${LEG_META[leg].name}: ${
                    telemetry.contacts[leg] ? "stance" : "swing"
                  }`}
                >
                  <span>L{leg}</span>
                  <i />
                </button>
              ))}
            </div>
          </section>

          <section className="telemetry-block leg-loads">
            <div className="block-label">
              <span>LEG LOAD <small>VISUAL EST.</small></span>
              <span>STANCE</span>
            </div>
            {LEG_IDS.map((leg) => {
              const load = telemetry.contacts[leg]
                ? 52 + jointActivity(telemetry.joints[leg]) * 38
                : 8;
              return (
                <div className="load-row" key={leg}>
                  <span>{LEG_META[leg].short}</span>
                  <div><i style={{ width: `${load}%` }} /></div>
                  <b>{Math.round(load)}</b>
                </div>
              );
            })}
          </section>

          <section className="dial-row">
            <MetricDial value={supportScore} label="SUPPORT" unit="%" tone="green" />
            <MetricDial
              value={Math.min(100, manipulability * 22)}
              label="MOBILITY"
              unit="%"
            />
          </section>

          <section className="telemetry-block selected-readout">
            <div className="block-label">
              <span>SELECTED LEG</span>
              <strong>L{selectedLeg} · {LEG_META[selectedLeg].short}</strong>
            </div>
            <div className="xyz-grid">
              <span>X<b>{(selectedFoot[0] * 1000).toFixed(1)}</b><small>mm</small></span>
              <span>Y<b>{(selectedFoot[1] * 1000).toFixed(1)}</b><small>mm</small></span>
              <span>Z<b>{(selectedFoot[2] * 1000).toFixed(1)}</b><small>mm</small></span>
            </div>
          </section>
        </aside>

        <section className="viewport-panel">
          <RobotViewport
            ref={viewportRef}
            gait={gait}
            gaitParameters={gaitParameters}
            motion={motion}
            bodyPose={bodyPose}
            jointTargets={jointTargets}
            playing={playing}
            showGrid={showGrid}
            showAxes={showAxes}
            resetToken={resetToken}
            stepToken={stepToken}
            onStatus={handleStatus}
            onTelemetry={handleTelemetry}
          />

          <div className="viewport-topline">
            <div>
              <span>SCENE / S6LR_001</span>
              <strong>{GAIT_LIBRARY[gait].label.toUpperCase()} · {playing ? "RUNNING" : "HOLD"}</strong>
            </div>
            <div className="viewport-tools">
              <IconButton
                icon={Grid3X3}
                label="Toggle floor grid"
                active={showGrid}
                onClick={() => setShowGrid((value) => !value)}
              />
              <IconButton
                icon={Orbit}
                label="Toggle joint axes"
                active={showAxes}
                onClick={() => setShowAxes((value) => !value)}
              />
              <IconButton
                icon={Maximize2}
                label="Reset camera"
                onClick={() => viewportRef.current?.resetCamera()}
              />
            </div>
          </div>

          <div className="view-presets" aria-label="Camera presets">
            {(
              [
                ["perspective", Camera, "Perspective"],
                ["top", Eye, "Top"],
                ["front", ArrowDown, "Front"],
                ["side", ArrowRight, "Side"],
              ] as Array<[ViewPreset, IconType, string]>
            ).map(([view, Icon, label]) => (
              <button
                type="button"
                key={view}
                onClick={() => viewportRef.current?.setView(view)}
                title={`${label} view`}
                aria-label={`${label} view`}
              >
                <Icon size={14} />
                <span>{label}</span>
              </button>
            ))}
          </div>

          <div className="viewport-scale">
            <span>100 mm</span>
            <i />
          </div>

          <div className="viewport-status">
            <span>TRIS <b>{robotStatus.loaded ? formatTriangles : "—"}</b></span>
            <span>UP <b>+Z</b></span>
            <span>FRAME <b>BODY</b></span>
          </div>

          {!robotStatus.loaded && (
            <div className="model-loader" role="status">
              <div className="loader-glyph"><Hexagon size={42} strokeWidth={1} /></div>
              <strong>{robotStatus.error ? "MODEL LOAD ERROR" : "ASSEMBLING S6LR"}</strong>
              <span>
                {robotStatus.error ??
                  `Loading original CAD meshes · ${Math.round(
                    robotStatus.loadingProgress * 100,
                  )}%`}
              </span>
              {!robotStatus.error && (
                <div className="loader-track">
                  <i style={{ width: `${robotStatus.loadingProgress * 100}%` }} />
                </div>
              )}
            </div>
          )}
        </section>

        <aside className="control-panel panel">
          {mode === "gait" && (
            <>
              <SectionHeader
                eyebrow="Motion planner"
                title="Gait control"
                action={<Footprints size={17} />}
              />
              <section className="control-section">
                <div className="control-label">
                  <span>GAIT PATTERN</span>
                  <b>{GAIT_LIBRARY[gait].code}</b>
                </div>
                <div className="gait-grid">
                  {(Object.keys(GAIT_LIBRARY) as GaitName[]).map((name) => {
                    const config = GAIT_LIBRARY[name];
                    return (
                      <button
                        type="button"
                        key={name}
                        className={gait === name ? "is-active" : ""}
                        onClick={() => chooseGait(name)}
                      >
                        <span>{config.code}</span>
                        <b>{config.label}</b>
                        <small>{config.description}</small>
                      </button>
                    );
                  })}
                </div>
              </section>

              <section className="transport-section">
                <button
                  type="button"
                  className={`run-button${playing ? " running" : ""}`}
                  onClick={() => setPlaying((value) => !value)}
                >
                  {playing ? <Pause size={16} fill="currentColor" /> : <Play size={16} fill="currentColor" />}
                  <span>{playing ? "PAUSE" : "RUN GAIT"}</span>
                </button>
                <IconButton
                  icon={StepForward}
                  label="Single kinematic step"
                  onClick={() => {
                    setPlaying(false);
                    setStepToken((token) => token + 1);
                  }}
                />
                <IconButton icon={RotateCcw} label="Reset simulator" onClick={resetSimulator} />
              </section>

              <section className="control-section direction-section">
                <div className="control-label">
                  <span>DIRECTION / TWIST</span>
                  <b>BODY</b>
                </div>
                <div className="direction-layout">
                  <button
                    type="button"
                    className={motion.forward > 0 && !motion.turn ? "active" : ""}
                    onClick={() => setDirection({ forward: 1, lateral: 0, turn: 0 })}
                    aria-label="Walk forward"
                  ><ArrowUp size={18} /></button>
                  <button
                    type="button"
                    className={motion.lateral < 0 && !motion.turn ? "active" : ""}
                    onClick={() => setDirection({ forward: 0, lateral: -1, turn: 0 })}
                    aria-label="Strafe left"
                  ><ArrowLeft size={18} /></button>
                  <button
                    type="button"
                    className="stop-motion"
                    onClick={() => setMotion({ forward: 0, lateral: 0, turn: 0 })}
                    aria-label="Stop motion"
                  ><Square size={12} fill="currentColor" /></button>
                  <button
                    type="button"
                    className={motion.lateral > 0 && !motion.turn ? "active" : ""}
                    onClick={() => setDirection({ forward: 0, lateral: 1, turn: 0 })}
                    aria-label="Strafe right"
                  ><ArrowRight size={18} /></button>
                  <button
                    type="button"
                    className={motion.forward < 0 && !motion.turn ? "active" : ""}
                    onClick={() => setDirection({ forward: -1, lateral: 0, turn: 0 })}
                    aria-label="Walk backward"
                  ><ArrowDown size={18} /></button>
                </div>
                <div className="turn-row">
                  <button
                    type="button"
                    className={motion.turn < 0 ? "active" : ""}
                    onClick={() => setMotion({ forward: 0, lateral: 0, turn: -1 })}
                  ><RotateCcw size={15} /> TURN LEFT <kbd>Q</kbd></button>
                  <button
                    type="button"
                    className={motion.turn > 0 ? "active" : ""}
                    onClick={() => setMotion({ forward: 0, lateral: 0, turn: 1 })}
                  >TURN RIGHT <kbd>E</kbd><RotateCw size={15} /></button>
                </div>
              </section>

              <section className="control-section parameter-stack">
                <div className="control-label">
                  <span>GAIT PARAMETERS</span>
                  <b>LIVE</b>
                </div>
                <RangeControl
                  label="Cycle speed"
                  value={gaitParameters.speed}
                  min={0.25}
                  max={2}
                  step={0.05}
                  unit="×"
                  onChange={(value) =>
                    setGaitParameters((current) => ({ ...current, speed: value }))
                  }
                />
                <RangeControl
                  label="Stride"
                  value={gaitParameters.stride}
                  min={0.005}
                  max={0.04}
                  step={0.001}
                  unit="mm"
                  format={(value) => (value * 1000).toFixed(0)}
                  onChange={(value) =>
                    setGaitParameters((current) => ({ ...current, stride: value }))
                  }
                />
                <RangeControl
                  label="Foot lift"
                  value={gaitParameters.lift}
                  min={0.004}
                  max={0.025}
                  step={0.001}
                  unit="mm"
                  format={(value) => (value * 1000).toFixed(0)}
                  onChange={(value) =>
                    setGaitParameters((current) => ({ ...current, lift: value }))
                  }
                />
                <RangeControl
                  label="Duty factor"
                  value={gaitParameters.duty}
                  min={0.45}
                  max={0.9}
                  step={0.01}
                  unit="%"
                  format={(value) => (value * 100).toFixed(0)}
                  onChange={(value) =>
                    setGaitParameters((current) => ({ ...current, duty: value }))
                  }
                />
              </section>

              <section className="phase-strip">
                <div className="control-label">
                  <span>LEG PHASES</span>
                  <b>{(telemetry.phase * 360).toFixed(0)}°</b>
                </div>
                <div>
                  {LEG_IDS.map((leg) => (
                    <span
                      key={leg}
                      className={telemetry.contacts[leg] ? "stance" : "swing"}
                      title={`Leg ${leg}: ${telemetry.contacts[leg] ? "stance" : "swing"}`}
                    >
                      L{leg}
                    </span>
                  ))}
                </div>
              </section>
            </>
          )}

          {mode === "joints" && (
            <>
              <SectionHeader
                eyebrow="Raw actuator space"
                title="Joint control"
                action={<SlidersHorizontal size={17} />}
              />
              <LegSelector selected={selectedLeg} onSelect={selectLeg} />
              <section className="control-section">
                <div className="switch-row">
                  <span>
                    <b>MIRROR PAIR</b>
                    <small>Maintain symmetric canonical angles</small>
                  </span>
                  <button
                    type="button"
                    className={`toggle-switch${mirror ? " on" : ""}`}
                    onClick={() => setMirror((value) => !value)}
                    aria-pressed={mirror}
                    aria-label="Mirror paired leg"
                  ><i /></button>
                </div>
              </section>
              <section className="control-section parameter-stack joint-stack">
                <div className="control-label">
                  <span>L{selectedLeg} RAW JOINTS</span>
                  <b>RAD / DEG</b>
                </div>
                {(["hip", "femur", "tibia"] as const).map((joint) => (
                  <RangeControl
                    key={joint}
                    label={joint === "hip" ? "Hip · yaw Z" : `${joint[0].toUpperCase()}${joint.slice(1)} · pitch X`}
                    value={jointTargets[selectedLeg][joint]}
                    min={JOINT_LIMITS[joint][0]}
                    max={JOINT_LIMITS[joint][1]}
                    step={0.005}
                    unit="°"
                    format={(value) => degrees(value).toFixed(1)}
                    onChange={(value) => updateJoint(joint, value)}
                  />
                ))}
              </section>
              <section className="joint-values">
                {(["hip", "femur", "tibia"] as const).map((joint) => (
                  <span key={joint}>
                    <small>{joint.toUpperCase()}</small>
                    <b>{selectedAngles[joint].toFixed(3)}</b>
                    <em>rad</em>
                  </span>
                ))}
              </section>
              <section className="control-section">
                <div className="control-label">
                  <span>CANONICAL SYMMETRY</span>
                  <b>S × QRAW</b>
                </div>
                <div className="canonical-readout">
                  <span>YAW <b>{degrees(selectedCanonical.hip).toFixed(1)}°</b></span>
                  <span>FEM <b>{degrees(selectedCanonical.femur).toFixed(1)}°</b></span>
                  <span>TIB <b>{degrees(selectedCanonical.tibia).toFixed(1)}°</b></span>
                </div>
              </section>
              <div className="action-row">
                <button
                  type="button"
                  onClick={() => {
                    const next = cloneJointMap(jointTargets);
                    next[selectedLeg] = { ...STANDING_JOINTS[selectedLeg] };
                    if (mirror) {
                      next[LEG_META[selectedLeg].pair] = {
                        ...STANDING_JOINTS[LEG_META[selectedLeg].pair],
                      };
                    }
                    setJointTargets(next);
                  }}
                ><RotateCcw size={14} /> RESET LEG</button>
                <button type="button" className="primary" onClick={() => setJointTargets(cloneJointMap())}>
                  <Check size={14} /> ALL STANCE
                </button>
              </div>
              <KinematicsReadout
                foot={selectedFoot}
                manipulability={manipulability}
                reachable={telemetry.reachability[selectedLeg]}
              />
            </>
          )}

          {mode === "ik" && (
            <>
              <SectionHeader
                eyebrow="Analytical solver"
                title="Forward / inverse"
                action={<Crosshair size={17} />}
              />
              <LegSelector selected={selectedLeg} onSelect={selectLeg} />
              <section className="control-section">
                <div className="switch-row">
                  <span>
                    <b>MIRROR TARGET</b>
                    <small>Reflect Y across the body centerline</small>
                  </span>
                  <button
                    type="button"
                    className={`toggle-switch${mirror ? " on" : ""}`}
                    onClick={() => setMirror((value) => !value)}
                    aria-pressed={mirror}
                    aria-label="Mirror IK target"
                  ><i /></button>
                </div>
              </section>
              <section className="control-section parameter-stack">
                <div className="control-label">
                  <span>FOOT TARGET · BODY FRAME</span>
                  <b>MM</b>
                </div>
                <RangeControl
                  label="X · longitudinal"
                  value={ikTarget.x}
                  min={-0.16}
                  max={0.16}
                  step={0.001}
                  unit="mm"
                  format={(value) => (value * 1000).toFixed(0)}
                  onChange={(value) => solveIkTarget({ ...ikTarget, x: value })}
                />
                <RangeControl
                  label="Y · outward"
                  value={ikTarget.outward}
                  min={0.045}
                  max={0.15}
                  step={0.001}
                  unit="mm"
                  format={(value) => (value * 1000).toFixed(0)}
                  onChange={(value) => solveIkTarget({ ...ikTarget, outward: value })}
                />
                <RangeControl
                  label="Z · vertical"
                  value={ikTarget.z}
                  min={-0.16}
                  max={-0.035}
                  step={0.001}
                  unit="mm"
                  format={(value) => (value * 1000).toFixed(0)}
                  onChange={(value) => solveIkTarget({ ...ikTarget, z: value })}
                />
              </section>
              <section className={`solver-result ${ikReachable ? "valid" : "invalid"}`}>
                <div>
                  {ikReachable ? <Check size={17} /> : <Info size={17} />}
                  <span>
                    <b>{ikReachable ? "STANCE BRANCH SOLVED" : "PROJECTED TO WORKSPACE"}</b>
                    <small>Residual {(ikResidual * 1000).toFixed(2)} mm</small>
                  </span>
                </div>
                <button type="button" onClick={() => solveIkTarget(ikTarget, false)}>
                  SOLVE
                </button>
              </section>
              <KinematicsReadout
                foot={selectedFoot}
                manipulability={manipulability}
                reachable={ikReachable}
              />
              <section className="geometry-note">
                <Braces size={16} />
                <div>
                  <b>STL-DERIVED CHAIN</b>
                  <span>Coxa 24.0 · Femur 56.46 · Tibia 77.13 mm</span>
                </div>
              </section>
              <section className="matrix-readout">
                <div className="control-label">
                  <span>JACOBIAN · M/S PER RAD/S</span>
                  <b>3 × 3</b>
                </div>
                <div>
                  {jacobian.flat().map((value, index) => (
                    <span key={index}>{value.toFixed(3)}</span>
                  ))}
                </div>
              </section>
            </>
          )}

          {mode === "body" && (
            <>
              <SectionHeader
                eyebrow="Planted-foot transform"
                title="Body pose"
                action={<Move3d size={17} />}
              />
              <section className="pose-visual">
                <div className="pose-crosshair">
                  <i style={{ transform: `rotate(${degrees(bodyPose.roll)}deg)` }} />
                  <span style={{ transform: `translateY(${bodyPose.pitch * 48}px)` }} />
                </div>
                <div>
                  <b>{(bodyPose.height * 1000).toFixed(0)} mm</b>
                  <span>BODY CLEARANCE</span>
                </div>
              </section>
              <section className="control-section parameter-stack">
                <div className="control-label">
                  <span>TRANSLATION</span>
                  <b>Z</b>
                </div>
                <RangeControl
                  label="Body height"
                  value={bodyPose.height}
                  min={0.07}
                  max={0.135}
                  step={0.001}
                  unit="mm"
                  format={(value) => (value * 1000).toFixed(0)}
                  onChange={(value) => updateBodyPose("height", value)}
                />
              </section>
              <section className="control-section parameter-stack">
                <div className="control-label">
                  <span>ORIENTATION</span>
                  <b>RPY</b>
                </div>
                <RangeControl
                  label="Roll · X"
                  value={bodyPose.roll}
                  min={radians(-14)}
                  max={radians(14)}
                  step={radians(0.25)}
                  unit="°"
                  format={(value) => degrees(value).toFixed(1)}
                  onChange={(value) => updateBodyPose("roll", value)}
                />
                <RangeControl
                  label="Pitch · Y"
                  value={bodyPose.pitch}
                  min={radians(-14)}
                  max={radians(14)}
                  step={radians(0.25)}
                  unit="°"
                  format={(value) => degrees(value).toFixed(1)}
                  onChange={(value) => updateBodyPose("pitch", value)}
                />
                <RangeControl
                  label="Yaw · Z"
                  value={bodyPose.yaw}
                  min={radians(-18)}
                  max={radians(18)}
                  step={radians(0.25)}
                  unit="°"
                  format={(value) => degrees(value).toFixed(1)}
                  onChange={(value) => updateBodyPose("yaw", value)}
                />
              </section>
              <section className="pose-presets">
                <button
                  type="button"
                  onClick={() => {
                    setBodyPose(INITIAL_BODY);
                    setJointTargets(solveBodyPose(INITIAL_BODY));
                  }}
                >LEVEL</button>
                <button
                  type="button"
                  onClick={() => {
                    const pose = { ...INITIAL_BODY, height: 0.078 };
                    setBodyPose(pose);
                    setJointTargets(solveBodyPose(pose));
                  }}
                >CROUCH</button>
                <button
                  type="button"
                  onClick={() => {
                    const pose = { ...INITIAL_BODY, roll: radians(9) };
                    setBodyPose(pose);
                    setJointTargets(solveBodyPose(pose));
                  }}
                >LEAN R</button>
                <button
                  type="button"
                  onClick={() => {
                    const pose = { ...INITIAL_BODY, pitch: radians(-9) };
                    setBodyPose(pose);
                    setJointTargets(solveBodyPose(pose));
                  }}
                >NOSE DN</button>
              </section>
              <section className="stability-note">
                <CircleGauge size={20} />
                <div>
                  <span>SUPPORT MARGIN</span>
                  <b>{supportScore >= 66 ? "NOMINAL" : "MARGINAL"} · {supportScore}%</b>
                </div>
              </section>
              <p className="scope-note">
                Body transforms preserve nominal planted feet through inverse
                rotation before solving each leg. This is geometric preview,
                not gravity or collision dynamics.
              </p>
            </>
          )}
        </aside>
      </div>

      <footer className="statusbar">
        <div>
          <span><i className="key">WASD</i> TRANSLATE</span>
          <span><i className="key">Q E</i> TURN</span>
          <span><i className="key">SPACE</i> RUN / PAUSE</span>
          <span><i className="key">R</i> RESET</span>
        </div>
        <div>
          <span>FRONT <b>−X</b></span>
          <span>RIGHT <b>+Y</b></span>
          <span>UP <b>+Z</b></span>
          <span className="kinematic-badge"><Zap size={11} /> KINEMATIC PREVIEW</span>
        </div>
      </footer>

      {toast && <div className="toast" role="status">{toast}</div>}
    </main>
  );
}

function LegSelector({
  selected,
  onSelect,
}: {
  selected: LegId;
  onSelect: (leg: LegId) => void;
}) {
  return (
    <section className="control-section">
      <div className="control-label">
        <span>LEG SELECT</span>
        <b>{LEG_META[selected].name.toUpperCase()}</b>
      </div>
      <div className="leg-selector">
        {LEG_IDS.map((leg) => (
          <button
            key={leg}
            type="button"
            className={selected === leg ? "is-active" : ""}
            onClick={() => onSelect(leg)}
          >
            <span>L{leg}</span>
            <b>{LEG_META[leg].short}</b>
          </button>
        ))}
      </div>
    </section>
  );
}

function KinematicsReadout({
  foot,
  manipulability,
  reachable,
}: {
  foot: [number, number, number];
  manipulability: number;
  reachable: boolean;
}) {
  return (
    <section className="kinematics-readout">
      <div className="control-label">
        <span>FORWARD KINEMATICS</span>
        <b className={reachable ? "valid-text" : "invalid-text"}>
          {reachable ? "VALID" : "LIMIT"}
        </b>
      </div>
      <div className="fk-values">
        {(["X", "Y", "Z"] as const).map((axis, index) => (
          <span key={axis}>
            <small>{axis}</small>
            <b>{(foot[index] * 1000).toFixed(1)}</b>
            <em>mm</em>
          </span>
        ))}
      </div>
      <div className="manipulability">
        <span>JACOBIAN VOLUME</span>
        <b>{manipulability.toFixed(3)} × 10⁻⁶ m³</b>
      </div>
    </section>
  );
}
