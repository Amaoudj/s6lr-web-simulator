"use client";

import {
  forwardRef,
  useEffect,
  useImperativeHandle,
  useRef,
} from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import {
  forwardKinematics,
  gaitCyclePeriod,
  LEG_IDS,
  solveGaitPose,
} from "../lib/kinematics";
import type {
  BodyPose,
  FrameTelemetry,
  GaitName,
  GaitParameters,
  JointMap,
  LegId,
  MotionCommand,
  RobotStatus,
  ViewportHandle,
  ViewPreset,
  WorldPose,
} from "../lib/simulator-types";

type RobotViewportProps = {
  gait: GaitName;
  gaitParameters: GaitParameters;
  motion: MotionCommand;
  bodyPose: BodyPose;
  jointTargets: JointMap;
  playing: boolean;
  showGrid: boolean;
  showAxes: boolean;
  resetToken: number;
  stepToken: number;
  onStatus: (status: RobotStatus) => void;
  onTelemetry: (telemetry: FrameTelemetry) => void;
};

type RuntimeProps = Pick<
  RobotViewportProps,
  | "gait"
  | "gaitParameters"
  | "motion"
  | "bodyPose"
  | "jointTargets"
  | "playing"
>;

const ROBOT_URL = "./models/s6lr/hexapod.urdf";
const TRAIL_CAPACITY = 700;

function sourceToScene(
  point: [number, number, number],
  height: number,
  world: WorldPose,
) {
  const cosine = Math.cos(world.yaw);
  const sine = Math.sin(world.yaw);
  const x = world.x + cosine * point[0] - sine * point[1];
  const y = world.y + sine * point[0] + cosine * point[1];
  return new THREE.Vector3(x, point[2] + height, -y);
}

function cameraPosition(view: ViewPreset) {
  switch (view) {
    case "top":
      return new THREE.Vector3(0, 0.48, 0.001);
    case "front":
      return new THREE.Vector3(-0.42, 0.12, 0);
    case "side":
      return new THREE.Vector3(0, 0.13, 0.42);
    default:
      return new THREE.Vector3(-0.3, 0.24, 0.34);
  }
}

export const RobotViewport = forwardRef<ViewportHandle, RobotViewportProps>(
  function RobotViewport(
    {
      gait,
      gaitParameters,
      motion,
      bodyPose,
      jointTargets,
      playing,
      showGrid,
      showAxes,
      resetToken,
      stepToken,
      onStatus,
      onTelemetry,
    },
    forwardedRef,
  ) {
    const hostRef = useRef<HTMLDivElement>(null);
    const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
    const controlsRef = useRef<OrbitControls | null>(null);
    const runtimeRef = useRef<RuntimeProps>({
      gait,
      gaitParameters,
      motion,
      bodyPose,
      jointTargets,
      playing,
    });
    const statusCallbackRef = useRef(onStatus);
    const telemetryCallbackRef = useRef(onTelemetry);
    const displayRef = useRef({ showGrid, showAxes });
    const phaseRef = useRef(0);
    const totalPhaseRef = useRef(0);
    const steppedPoseRef = useRef(false);
    const pendingStepFractionRef = useRef(0);
    const trailResetRef = useRef(false);
    const worldPoseRef = useRef<WorldPose>({
      x: 0,
      y: 0,
      yaw: 0,
      distance: 0,
      speed: 0,
    });
    const resetCameraRef = useRef<() => void>(() => undefined);
    const viewRef = useRef<(view: ViewPreset) => void>(() => undefined);

    useEffect(() => {
      runtimeRef.current = {
        gait,
        gaitParameters,
        motion,
        bodyPose,
        jointTargets,
        playing,
      };
      if (playing) steppedPoseRef.current = true;
    }, [gait, gaitParameters, motion, bodyPose, jointTargets, playing]);

    useEffect(() => {
      statusCallbackRef.current = onStatus;
      telemetryCallbackRef.current = onTelemetry;
    }, [onStatus, onTelemetry]);

    useEffect(() => {
      displayRef.current = { showGrid, showAxes };
    }, [showGrid, showAxes]);

    useEffect(() => {
      phaseRef.current = 0;
      totalPhaseRef.current = 0;
      steppedPoseRef.current = false;
      pendingStepFractionRef.current = 0;
      worldPoseRef.current = {
        x: 0,
        y: 0,
        yaw: 0,
        distance: 0,
        speed: 0,
      };
      trailResetRef.current = true;
      resetCameraRef.current();
    }, [resetToken]);

    useEffect(() => {
      if (stepToken === 0) return;
      phaseRef.current = (phaseRef.current + 1 / 30) % 1;
      totalPhaseRef.current += 1 / 30;
      pendingStepFractionRef.current += 1 / 30;
      steppedPoseRef.current = true;
    }, [stepToken]);

    useImperativeHandle(
      forwardedRef,
      () => ({
        setView: (view) => viewRef.current(view),
        resetCamera: () => resetCameraRef.current(),
      }),
      [],
    );

    useEffect(() => {
      const host = hostRef.current;
      if (!host) return;

      THREE.Cache.enabled = true;
      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0xeeeeea);
      scene.fog = new THREE.FogExp2(0xeeeeea, 0.32);

      const camera = new THREE.PerspectiveCamera(38, 1, 0.005, 8);
      camera.position.copy(cameraPosition("perspective"));
      cameraRef.current = camera;

      const renderer = new THREE.WebGLRenderer({
        antialias: true,
        alpha: false,
        powerPreference: "high-performance",
      });
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.toneMapping = THREE.ACESFilmicToneMapping;
      renderer.toneMappingExposure = 1.08;
      renderer.shadowMap.enabled = true;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.6));
      host.appendChild(renderer.domElement);

      const controls = new OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.075;
      controls.minDistance = 0.18;
      controls.maxDistance = 2.2;
      controls.maxPolarAngle = Math.PI * 0.49;
      controls.target.set(0, 0.045, 0);
      controlsRef.current = controls;
      const followCenter = new THREE.Vector3(0, 0.045, 0);

      const setView = (view: ViewPreset) => {
        const world = worldPoseRef.current;
        const center = new THREE.Vector3(world.x, 0.045, -world.y);
        camera.position
          .copy(cameraPosition(view))
          .add(new THREE.Vector3(world.x, 0, -world.y));
        camera.up.set(0, 1, 0);
        if (view === "top") camera.up.set(0, 0, -1);
        controls.target.copy(center);
        followCenter.copy(center);
        controls.update();
      };
      viewRef.current = setView;
      resetCameraRef.current = () => setView("perspective");

      const ambient = new THREE.HemisphereLight(0xffffff, 0x4b514e, 2.65);
      scene.add(ambient);
      const keyLight = new THREE.DirectionalLight(0xfff7e8, 4.2);
      keyLight.position.set(-0.28, 0.5, 0.34);
      keyLight.castShadow = true;
      keyLight.shadow.mapSize.set(2048, 2048);
      keyLight.shadow.camera.near = 0.05;
      keyLight.shadow.camera.far = 1.5;
      keyLight.shadow.camera.left = -0.35;
      keyLight.shadow.camera.right = 0.35;
      keyLight.shadow.camera.top = 0.35;
      keyLight.shadow.camera.bottom = -0.35;
      scene.add(keyLight);
      const rimLight = new THREE.DirectionalLight(0xff6a3d, 1.55);
      rimLight.position.set(0.28, 0.2, -0.25);
      scene.add(rimLight);

      const floor = new THREE.Mesh(
        new THREE.PlaneGeometry(6, 6),
        new THREE.ShadowMaterial({ color: 0x2c302e, opacity: 0.16 }),
      );
      floor.rotation.x = -Math.PI / 2;
      floor.position.y = -0.001;
      floor.receiveShadow = true;
      scene.add(floor);

      const grid = new THREE.GridHelper(6, 120, 0x8d918e, 0xc9cac5);
      const gridMaterials = Array.isArray(grid.material)
        ? grid.material
        : [grid.material];
      for (const material of gridMaterials) {
        material.transparent = true;
        material.opacity = 0.42;
      }
      grid.position.y = 0;
      grid.visible = displayRef.current.showGrid;
      scene.add(grid);

      const originRing = new THREE.Mesh(
        new THREE.RingGeometry(0.175, 0.1765, 96),
        new THREE.MeshBasicMaterial({
          color: 0xa9aaa4,
          transparent: true,
          opacity: 0.65,
          side: THREE.DoubleSide,
        }),
      );
      originRing.rotation.x = -Math.PI / 2;
      originRing.position.y = 0.0005;
      scene.add(originRing);

      const contactMarkers = {} as Record<LegId, THREE.Mesh>;
      for (const leg of LEG_IDS) {
        const marker = new THREE.Mesh(
          new THREE.RingGeometry(0.0065, 0.009, 28),
          new THREE.MeshBasicMaterial({
            color: 0x27a071,
            transparent: true,
            opacity: 0.9,
            side: THREE.DoubleSide,
          }),
        );
        marker.rotation.x = -Math.PI / 2;
        marker.position.y = 0.0015;
        scene.add(marker);
        contactMarkers[leg] = marker;
      }

      const supportGeometry = new THREE.BufferGeometry();
      supportGeometry.setAttribute(
        "position",
        new THREE.BufferAttribute(new Float32Array(6 * 3), 3),
      );
      const supportPolygon = new THREE.LineLoop(
        supportGeometry,
        new THREE.LineBasicMaterial({
          color: 0x1f9a70,
          transparent: true,
          opacity: 0.72,
        }),
      );
      supportPolygon.frustumCulled = false;
      supportPolygon.visible = false;
      scene.add(supportPolygon);

      const supportCenter = new THREE.Mesh(
        new THREE.CircleGeometry(0.004, 24),
        new THREE.MeshBasicMaterial({
          color: 0x1f9a70,
          transparent: true,
          opacity: 0.9,
          side: THREE.DoubleSide,
        }),
      );
      supportCenter.rotation.x = -Math.PI / 2;
      supportCenter.visible = false;
      scene.add(supportCenter);

      const trailPositions = new Float32Array(TRAIL_CAPACITY * 3);
      const trailGeometry = new THREE.BufferGeometry();
      trailGeometry.setAttribute(
        "position",
        new THREE.BufferAttribute(trailPositions, 3),
      );
      trailGeometry.setDrawRange(0, 0);
      const worldTrail = new THREE.Line(
        trailGeometry,
        new THREE.LineBasicMaterial({
          color: 0xf45b35,
          transparent: true,
          opacity: 0.78,
        }),
      );
      worldTrail.frustumCulled = false;
      scene.add(worldTrail);
      let trailCount = 0;
      let lastTrailPoint = new THREE.Vector3(Number.POSITIVE_INFINITY, 0, 0);

      const velocityArrow = new THREE.ArrowHelper(
        new THREE.Vector3(-1, 0, 0),
        new THREE.Vector3(0, 0.006, 0),
        0.06,
        0xf45b35,
        0.014,
        0.008,
      );
      velocityArrow.visible = false;
      scene.add(velocityArrow);

      const axisMapRoot = new THREE.Group();
      axisMapRoot.rotation.x = -Math.PI / 2;
      scene.add(axisMapRoot);
      const bodySourceFrame = new THREE.Group();
      axisMapRoot.add(bodySourceFrame);

      let robot: URDFRobot | null = null;
      const jointAxes: THREE.AxesHelper[] = [];
      let triangleCount = 0;
      const geometryCache = new Map<string, Promise<THREE.BufferGeometry>>();
      const loadingManager = new THREE.LoadingManager();

      const publishStatus = (
        patch: Partial<RobotStatus> & Pick<RobotStatus, "loaded">,
      ) => {
        statusCallbackRef.current({
          loaded: patch.loaded,
          loadingProgress: patch.loadingProgress ?? (patch.loaded ? 1 : 0),
          error: patch.error ?? null,
          fps: patch.fps ?? 0,
          triangleCount: patch.triangleCount ?? triangleCount,
        });
      };

      loadingManager.onProgress = (_url, loaded, total) => {
        publishStatus({
          loaded: false,
          loadingProgress: total > 0 ? loaded / total : 0,
        });
      };
      loadingManager.onError = (url) => {
        publishStatus({
          loaded: false,
          error: `Could not load model asset: ${decodeURI(url).split("/").pop()}`,
        });
      };

      const bodyMaterial = new THREE.MeshStandardMaterial({
        color: 0x090c0c,
        roughness: 0.43,
        metalness: 0.48,
        side: THREE.DoubleSide,
      });

      const styleRobot = () => {
        if (!robot) return;
        triangleCount = 0;
        robot.traverse((object) => {
          if (!(object instanceof THREE.Mesh)) return;
          const mesh = object as THREE.Mesh;
          const geometry = mesh.geometry as THREE.BufferGeometry;
          const position = geometry.getAttribute("position");
          if (geometry.index) triangleCount += geometry.index.count / 3;
          else if (position) triangleCount += position.count / 3;
          mesh.material = bodyMaterial;
          mesh.castShadow = true;
          mesh.receiveShadow = true;
        });
        publishStatus({
          loaded: true,
          loadingProgress: 1,
          triangleCount: Math.round(triangleCount),
        });
      };

      loadingManager.onLoad = () => {
        requestAnimationFrame(styleRobot);
      };

      const urdfLoader = new URDFLoader(loadingManager);
      urdfLoader.packages = "";
      urdfLoader.loadMeshCb = (path, manager, material, done) => {
        let geometryPromise = geometryCache.get(path);
        if (!geometryPromise) {
          geometryPromise = new STLLoader(manager).loadAsync(path).then((geometry) => {
            geometry.computeVertexNormals();
            geometry.computeBoundingSphere();
            return geometry;
          });
          geometryCache.set(path, geometryPromise);
        }
        geometryPromise
          .then((geometry) => done(new THREE.Mesh(geometry, material)))
          .catch((error: Error) => done(new THREE.Object3D(), error));
      };
      urdfLoader.load(
        ROBOT_URL,
        (loadedRobot) => {
          robot = loadedRobot;
          robot.name = "S6LR";
          bodySourceFrame.add(robot);
          for (const leg of LEG_IDS) {
            for (const segment of ["hip", "femur", "tibia"]) {
              const joint = robot.joints[`${segment}_joint_${leg}`];
              if (!joint) continue;
              const helper = new THREE.AxesHelper(0.015);
              helper.visible = displayRef.current.showAxes;
              joint.add(helper);
              jointAxes.push(helper);
            }
          }
        },
        undefined,
        (error) => {
          publishStatus({
            loaded: false,
            error: error instanceof Error ? error.message : "Robot model failed to load.",
          });
        },
      );

      let lastTime = performance.now();
      let telemetryTime = 0;
      let fpsWindowStart = lastTime;
      let fpsFrames = 0;
      let currentFps = 60;
      let animationFrame = 0;
      const animate = (now: number) => {
        animationFrame = requestAnimationFrame(animate);
        const delta = Math.min(0.05, Math.max(0, (now - lastTime) / 1000));
        lastTime = now;
        fpsFrames += 1;
        if (now - fpsWindowStart >= 1000) {
          currentFps = (fpsFrames * 1000) / (now - fpsWindowStart);
          fpsFrames = 0;
          fpsWindowStart = now;
        }

        const runtime = runtimeRef.current;
        const period = gaitCyclePeriod(runtime.gait, runtime.gaitParameters.speed);
        let cycleFraction = 0;
        if (runtime.playing) {
          const phaseDelta = delta / period;
          totalPhaseRef.current += phaseDelta;
          phaseRef.current = totalPhaseRef.current % 1;
          cycleFraction = phaseDelta;
        } else if (pendingStepFractionRef.current > 0) {
          cycleFraction = pendingStepFractionRef.current;
          pendingStepFractionRef.current = 0;
        }

        const world = worldPoseRef.current;
        if (cycleFraction > 0) {
          const forwardDistance =
            runtime.motion.forward * runtime.gaitParameters.stride * cycleFraction;
          const lateralDistance =
            runtime.motion.lateral * runtime.gaitParameters.stride * cycleFraction;
          const cosine = Math.cos(world.yaw);
          const sine = Math.sin(world.yaw);
          const dx = cosine * -forwardDistance - sine * lateralDistance;
          const dy = sine * -forwardDistance + cosine * lateralDistance;
          world.x += dx;
          world.y += dy;
          world.yaw +=
            runtime.motion.turn *
            (runtime.gaitParameters.stride / 0.1) *
            cycleFraction;
          world.distance += Math.hypot(dx, dy);
        }
        world.speed = runtime.playing
          ? (runtime.gaitParameters.stride / period) *
            Math.hypot(runtime.motion.forward, runtime.motion.lateral)
          : 0;

        let activeJoints = runtime.jointTargets;
        let contacts = {} as Record<LegId, boolean>;
        let reachability = {} as Record<LegId, boolean>;
        let feet = {} as Record<LegId, [number, number, number]>;

        if (runtime.playing || steppedPoseRef.current) {
          const gaitPose = solveGaitPose(
            runtime.gait,
            phaseRef.current,
            runtime.gaitParameters,
            runtime.motion,
            runtime.bodyPose,
          );
          activeJoints = gaitPose.joints;
          contacts = gaitPose.contacts;
          reachability = gaitPose.reachability;
          feet = gaitPose.feet;
        } else {
          for (const leg of LEG_IDS) {
            const foot = forwardKinematics(leg, activeJoints[leg]);
            feet[leg] = foot;
            contacts[leg] =
              Math.abs(foot[2] + runtime.bodyPose.height) < 0.012;
            reachability[leg] = true;
          }
        }

        axisMapRoot.position.y = runtime.bodyPose.height;
        bodySourceFrame.position.set(world.x, world.y, 0);
        bodySourceFrame.rotation.set(
          runtime.bodyPose.roll,
          runtime.bodyPose.pitch,
          world.yaw + runtime.bodyPose.yaw,
          "ZYX",
        );

        const supportPoints: THREE.Vector3[] = [];
        if (robot) {
          for (const leg of LEG_IDS) {
            const q = activeJoints[leg];
            robot.setJointValue(`hip_joint_${leg}`, q.hip);
            robot.setJointValue(`femur_joint_${leg}`, q.femur);
            robot.setJointValue(`tibia_joint_${leg}`, q.tibia);
            const marker = contactMarkers[leg];
            marker.position.copy(
              sourceToScene(feet[leg], runtime.bodyPose.height, world),
            );
            marker.position.y = Math.max(0.0015, marker.position.y);
            const markerMaterial = marker.material as THREE.MeshBasicMaterial;
            markerMaterial.color.setHex(contacts[leg] ? 0x238d67 : 0xf45b35);
            markerMaterial.opacity = contacts[leg] ? 0.85 : 0.48;
            if (contacts[leg]) supportPoints.push(marker.position.clone());
          }
        }

        if (supportPoints.length >= 3) {
          const centroid = supportPoints
            .reduce((sum, point) => sum.add(point), new THREE.Vector3())
            .multiplyScalar(1 / supportPoints.length);
          supportPoints.sort(
            (a, b) =>
              Math.atan2(a.z - centroid.z, a.x - centroid.x) -
              Math.atan2(b.z - centroid.z, b.x - centroid.x),
          );
          const positions = supportGeometry.getAttribute(
            "position",
          ) as THREE.BufferAttribute;
          supportPoints.forEach((point, index) => {
            positions.setXYZ(index, point.x, 0.0025, point.z);
          });
          positions.needsUpdate = true;
          supportGeometry.setDrawRange(0, supportPoints.length);
          supportPolygon.visible = true;
          supportCenter.position.set(centroid.x, 0.003, centroid.z);
          supportCenter.visible = true;
        } else {
          supportPolygon.visible = false;
          supportCenter.visible = false;
        }

        const bodyScenePosition = new THREE.Vector3(
          world.x,
          runtime.bodyPose.height,
          -world.y,
        );
        if (trailResetRef.current) {
          trailCount = 0;
          trailGeometry.setDrawRange(0, 0);
          lastTrailPoint = new THREE.Vector3(Number.POSITIVE_INFINITY, 0, 0);
          followCenter.set(0, 0.045, 0);
          trailResetRef.current = false;
        }
        const groundBodyPoint = new THREE.Vector3(world.x, 0.0035, -world.y);
        if (groundBodyPoint.distanceTo(lastTrailPoint) >= 0.0035) {
          if (trailCount === TRAIL_CAPACITY) {
            trailPositions.copyWithin(0, 3);
            trailCount -= 1;
          }
          trailPositions[trailCount * 3] = groundBodyPoint.x;
          trailPositions[trailCount * 3 + 1] = groundBodyPoint.y;
          trailPositions[trailCount * 3 + 2] = groundBodyPoint.z;
          trailCount += 1;
          (
            trailGeometry.getAttribute("position") as THREE.BufferAttribute
          ).needsUpdate = true;
          trailGeometry.setDrawRange(0, trailCount);
          lastTrailPoint.copy(groundBodyPoint);
        }

        const commandMagnitude = Math.hypot(
          runtime.motion.forward,
          runtime.motion.lateral,
        );
        if (commandMagnitude > 1e-4 && (runtime.playing || steppedPoseRef.current)) {
          const cosine = Math.cos(world.yaw);
          const sine = Math.sin(world.yaw);
          const vx =
            cosine * -runtime.motion.forward - sine * runtime.motion.lateral;
          const vy =
            sine * -runtime.motion.forward + cosine * runtime.motion.lateral;
          velocityArrow.position.copy(bodyScenePosition);
          velocityArrow.position.y = 0.017;
          velocityArrow.setDirection(
            new THREE.Vector3(vx, 0, -vy).normalize(),
          );
          velocityArrow.setLength(
            0.045 + Math.min(0.04, world.speed * 1.2),
            0.014,
            0.008,
          );
          velocityArrow.visible = true;
        } else {
          velocityArrow.visible = false;
        }

        const nextFollow = new THREE.Vector3(world.x, 0.045, -world.y);
        const followDelta = nextFollow.clone().sub(followCenter);
        if (followDelta.lengthSq() > 0) {
          camera.position.add(followDelta);
          controls.target.add(followDelta);
          followCenter.copy(nextFollow);
        }

        for (const helper of jointAxes) {
          helper.visible = displayRef.current.showAxes;
        }
        grid.visible = displayRef.current.showGrid;
        controls.update();
        renderer.render(scene, camera);

        if (now - telemetryTime >= 120) {
          telemetryTime = now;
          telemetryCallbackRef.current({
            joints: activeJoints,
            contacts,
            reachability,
            phase: phaseRef.current,
            cycle: Math.floor(totalPhaseRef.current),
            fps: currentFps,
            world: { ...world },
          });
        }
      };
      animationFrame = requestAnimationFrame(animate);

      const resize = () => {
        const width = Math.max(1, host.clientWidth);
        const height = Math.max(1, host.clientHeight);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height, false);
      };
      const resizeObserver = new ResizeObserver(resize);
      resizeObserver.observe(host);
      resize();

      return () => {
        cancelAnimationFrame(animationFrame);
        resizeObserver.disconnect();
        controls.dispose();
        renderer.dispose();
        renderer.domElement.remove();
        const disposed = new Set<THREE.BufferGeometry>();
        if (robot) {
          robot.traverse((object) => {
            if (object instanceof THREE.Mesh && !disposed.has(object.geometry)) {
              object.geometry.dispose();
              disposed.add(object.geometry);
            }
          });
        }
        bodyMaterial.dispose();
        supportGeometry.dispose();
        trailGeometry.dispose();
        cameraRef.current = null;
        controlsRef.current = null;
      };
    }, []);

    useEffect(() => {
      const sceneRuntime = runtimeRef.current;
      if (!sceneRuntime.playing) steppedPoseRef.current = false;
      sceneRuntime.jointTargets = jointTargets;
    }, [jointTargets]);

    return (
      <div className="viewport-canvas" ref={hostRef} aria-label="Interactive S6LR 3D robot viewport">
        <noscript>This simulator requires JavaScript and WebGL.</noscript>
      </div>
    );
  },
);
