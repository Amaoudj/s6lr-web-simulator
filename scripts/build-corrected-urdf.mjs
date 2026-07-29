import { mkdirSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";

const torsoMeshes = [
  "vim_top_cover.stl",
  "vim base.stl",
  "main body mid.stl",
  "main body lower.stl",
  "lower base.stl",
  "camera holder A.stl",
  "camera holder B.stl",
  "motor pitch roll holder.stl",
  "base roll.stl",
  "hip motor.stl",
];

const femurMeshes = [
  "Servo femur.stl",
  "Servo tibia.stl",
  "femur frame maini right.stl",
];

const tibiaMeshes = [
  "tibia frame A.stl",
  "tibia link A.stl",
  "tibia link B.stl",
  "tibia base.stl",
];

const stations = {
  1: [-0.05, -0.0035, 1],
  2: [-0.05, 0.0035, -1],
  3: [0, 0, 1],
  4: [0, 0, -1],
  5: [0.05, -0.0035, 1],
  6: [0.05, 0.0035, -1],
};

const number = (value) => {
  if (Math.abs(value) < 1e-12) return "0";
  return Number(value.toFixed(6)).toString();
};

const vector = (values) => values.map(number).join(" ");

const visual = (name, filename, origin = [0, 0, 0], side = 1) => `
    <visual name="${name}">
      <origin xyz="${vector(origin)}" rpy="0 0 0"/>
      <geometry>
        <mesh filename="xml_Files/${filename}" scale="0.001 ${number(
          0.001 * side,
        )} 0.001"/>
      </geometry>
      <material name="robot_black"/>
    </visual>`;

const sections = [
  `<?xml version="1.0"?>
<!--
  Browser URDF generated from Tracking_controller/xml/S6LR_v1.xml.
  MJCF meshes are authored in parent CAD frames. Each visual offset below is
  the negative of its true hinge pivot so the mesh rotates about the same axis
  as the controller model without the previous double translation.
-->
<robot name="s6lr_tracking_controller">
  <material name="robot_black">
    <color rgba="0.025 0.03 0.03 1"/>
  </material>
  <link name="torso">
${torsoMeshes
  .map((mesh, index) => visual(`torso_${index + 1}`, mesh))
  .join("\n")}
  </link>`,
];

for (let leg = 1; leg <= 6; leg += 1) {
  const [stationX, stationY, side] = stations[leg];
  const hipAnchor = [0, side * 0.028, -0.0415];
  const femurAnchor = [0, side * 0.052, -0.03];
  const tibiaAnchor = [0, side * 0.0685, 0.024];
  const hipOrigin = [
    stationX + hipAnchor[0],
    stationY + hipAnchor[1],
    hipAnchor[2],
  ];
  const femurOrigin = femurAnchor.map((value, axis) => value - hipAnchor[axis]);
  const tibiaOrigin = tibiaAnchor.map((value, axis) => value - femurAnchor[axis]);
  const hipVisualOrigin = hipAnchor.map((value) => -value);
  const femurVisualOrigin = femurAnchor.map((value) => -value);
  const tibiaVisualOrigin = tibiaAnchor.map((value) => -value);

  sections.push(`
  <!-- Leg ${leg}: ${side === 1 ? "right" : "left"} ${
    leg <= 2 ? "front" : leg <= 4 ? "middle" : "rear"
  } -->
  <joint name="hip_joint_${leg}" type="revolute">
    <parent link="torso"/>
    <child link="hip_${leg}"/>
    <origin xyz="${vector(hipOrigin)}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="5" velocity="3"/>
  </joint>
  <link name="hip_${leg}">
${visual(
  `hip_frame_${leg}`,
  "hip frame right mid.stl",
  hipVisualOrigin,
  side,
)}
  </link>

  <joint name="femur_joint_${leg}" type="revolute">
    <parent link="hip_${leg}"/>
    <child link="femur_${leg}"/>
    <origin xyz="${vector(femurOrigin)}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3" upper="3" effort="5" velocity="3"/>
  </joint>
  <link name="femur_${leg}">
${femurMeshes
  .map((mesh, index) =>
    visual(`femur_${leg}_${index + 1}`, mesh, femurVisualOrigin, side),
  )
  .join("\n")}
  </link>

  <joint name="tibia_joint_${leg}" type="revolute">
    <parent link="femur_${leg}"/>
    <child link="tibia_${leg}"/>
    <origin xyz="${vector(tibiaOrigin)}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3" upper="3" effort="5" velocity="3"/>
  </joint>
  <link name="tibia_${leg}">
${tibiaMeshes
  .map((mesh, index) =>
    visual(`tibia_${leg}_${index + 1}`, mesh, tibiaVisualOrigin, side),
  )
  .join("\n")}
  </link>`);
}

sections.push("</robot>\n");
const urdf = sections.join("\n");
const outputs = [
  resolve("robot-source/hexapod.urdf"),
  resolve("public/models/s6lr/hexapod.urdf"),
];

for (const output of outputs) {
  mkdirSync(dirname(output), { recursive: true });
  writeFileSync(output, urdf);
  console.log(`Wrote ${output}`);
}
