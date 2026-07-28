# S6LR Web Simulator

An interactive browser-based kinematics lab for the S6LR six-legged robot. The
viewer uses the original CAD STL meshes from the research workspace and exposes
all 18 actuated joints, analytical forward/inverse kinematics, body pose,
direction control, and four gait schedules.

## Included controls

- Tripod, ripple, wave, and tetrapod gait schedules
- Start, pause, single-step, reset, stride, speed, lift, and duty-factor controls
- Direction pad plus `W/A/S/D`, `Q/E`, and space-bar keyboard controls
- Manual hip/femur/tibia control for each leg
- Per-leg and mirrored inverse-kinematics target control
- Body height, roll, pitch, and yaw
- Perspective, top, front, and side camera presets
- Simulated contact/activity telemetry and exact joint/foot readouts
- Pose JSON export and import

## Model provenance and scope

The model is copied from `../xml_Files` and the valid project URDF at
`../pybullet/hexapod.urdf`. Untouched copies of the URDF, MJCF, and all 18
active STL dependencies are preserved in `robot-source`. Browser-served copies
live in `public/models/s6lr`; only the two six-times-instanced servo meshes are
decimated for faster loading. Their dimensions and kinematic pivots remain
unchanged.

This application is a kinematic preview, not a rigid-body dynamics solver.
Torque and contact values shown in the interface are clearly marked as
estimated visual telemetry. The supplied MJCF masses, actuator ranges, and some
joint limits are not physically calibrated.

## Run locally

Node.js 22 or newer is required.

```bash
npm install
npm run dev
```

Then open `http://127.0.0.1:3000`.

## Validate

```bash
npm run typecheck
npm run test:kinematics
npm run check
npm run build
```

The automated browser interaction check can be run against a production server:

```bash
npm run start
npm run smoke:browser
```

## Rebuild optimized meshes

```bash
python -m pip install -r scripts/requirements-mesh.txt
python scripts/optimize_meshes.py
```
