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

function sourceToScene(point: [number, number, number], height: number) {
  return new THREE.Vector3(point[0], point[2] + height, -point[1]);
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
      resetCameraRef.current();
    }, [resetToken]);

    useEffect(() => {
      if (stepToken === 0) return;
      phaseRef.current = (phaseRef.current + 1 / 30) % 1;
      totalPhaseRef.current += 1 / 30;
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
      scene.background = new THREE.Color(0xe8e8e2);
      scene.fog = new THREE.FogExp2(0xe8e8e2, 1.25);

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
      controls.maxDistance = 1.35;
      controls.maxPolarAngle = Math.PI * 0.49;
      controls.target.set(0, 0.045, 0);
      controlsRef.current = controls;

      const setView = (view: ViewPreset) => {
        camera.position.copy(cameraPosition(view));
        camera.up.set(0, 1, 0);
        if (view === "top") camera.up.set(0, 0, -1);
        controls.target.set(0, 0.045, 0);
        controls.update();
      };
      viewRef.current = setView;
      resetCameraRef.current = () => setView("perspective");

      const ambient = new THREE.HemisphereLight(0xffffff, 0x303633, 2.15);
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
      const rimLight = new THREE.DirectionalLight(0xff6a3d, 1.15);
      rimLight.position.set(0.28, 0.2, -0.25);
      scene.add(rimLight);

      const floor = new THREE.Mesh(
        new THREE.PlaneGeometry(2.4, 2.4),
        new THREE.ShadowMaterial({ color: 0x2c302e, opacity: 0.16 }),
      );
      floor.rotation.x = -Math.PI / 2;
      floor.position.y = -0.001;
      floor.receiveShadow = true;
      scene.add(floor);

      const grid = new THREE.GridHelper(1.2, 48, 0x9d9e98, 0xc8c8c1);
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
        color: 0xd8dad5,
        roughness: 0.48,
        metalness: 0.28,
      });
      const darkMaterial = new THREE.MeshStandardMaterial({
        color: 0x202525,
        roughness: 0.36,
        metalness: 0.55,
      });
      const accentMaterial = new THREE.MeshStandardMaterial({
        color: 0xf45b35,
        roughness: 0.38,
        metalness: 0.36,
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
          const incoming = Array.isArray(mesh.material)
            ? mesh.material[0]
            : mesh.material;
          const color =
            incoming && "color" in incoming
              ? (incoming as THREE.MeshStandardMaterial).color
              : null;
          if (color && color.b > color.r * 1.35 && color.b > color.g * 1.35) {
            mesh.material = accentMaterial;
          } else if (color && color.r + color.g + color.b < 0.8) {
            mesh.material = darkMaterial;
          } else {
            mesh.material = bodyMaterial;
          }
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
        if (runtime.playing) {
          const period = gaitCyclePeriod(runtime.gait, runtime.gaitParameters.speed);
          const phaseDelta = delta / period;
          totalPhaseRef.current += phaseDelta;
          phaseRef.current = totalPhaseRef.current % 1;
        }

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
        bodySourceFrame.rotation.set(
          runtime.bodyPose.roll,
          runtime.bodyPose.pitch,
          runtime.bodyPose.yaw,
          "ZYX",
        );

        if (robot) {
          for (const leg of LEG_IDS) {
            const q = activeJoints[leg];
            robot.setJointValue(`hip_joint_${leg}`, q.hip);
            robot.setJointValue(`femur_joint_${leg}`, q.femur);
            robot.setJointValue(`tibia_joint_${leg}`, q.tibia);
            const marker = contactMarkers[leg];
            marker.position.copy(
              sourceToScene(feet[leg], runtime.bodyPose.height),
            );
            marker.position.y = Math.max(0.0015, marker.position.y);
            const markerMaterial = marker.material as THREE.MeshBasicMaterial;
            markerMaterial.color.setHex(contacts[leg] ? 0x238d67 : 0xf45b35);
            markerMaterial.opacity = contacts[leg] ? 0.85 : 0.48;
          }
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
        darkMaterial.dispose();
        accentMaterial.dispose();
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
