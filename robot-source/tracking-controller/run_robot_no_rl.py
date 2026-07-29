import numpy as np
import mujoco
import mujoco.viewer
import gymnasium as gym
from gymnasium import spaces
import os
import time

from locomotion import Locomotion
from oscillator import Oscillator
from scipy.spatial.transform import Rotation as R


def wrap_to_pi(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def model_yaw_to_robot_heading_deg(model_yaw_deg):
    return np.degrees(wrap_to_pi(np.radians(model_yaw_deg) + np.pi))


class SingleLegRobotSim:
    def __init__(self, render_mode=True):
        self.render_mode = render_mode
        self.xml_path = "xml/S6LR_v1.xml"
        self.locomotion = Locomotion()
        self.oscillator = Oscillator()
        self.xml_path = "xml/S6LR_v1.xml"

        if not os.path.exists(self.xml_path):
            raise FileNotFoundError(f"XML file not found: {self.xml_path}")

        # Load model
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.model.opt.gravity = np.array([0, 0, -9.81])
        self.model.opt.timestep = 0.001
        self.model.opt.iterations = 1

        self.data = mujoco.MjData(self.model)

        # Viewer
        self.viewer = None

        # Robot initial state
        self.initial_height = 1.2

        self.initial_qpos = np.array(
            [0.74145681, -1.15066357,  1.09948703,
             -0.8149835,   1.15285511, -1.10027548,
             -0.00680523, -1.15695523,  0.99990021,
              0.06380703,  1.15762575, -1.00050958,
             -0.70499348, -1.14952721,  1.09352862,
              0.75207264,  1.14686083, -1.09545775],
            dtype=float
        )

        self.init_ctrl = np.array(
            [69.47382762, -118.49308143,  109.97924008,
             -69.47382762,  118.49308143, -109.97924008,
              0.,         -119.61903995,  100.19416334,
              0.,          119.61903995, -100.19416334,
             -69.47382762, -118.49308143,  109.97924008,
              69.47382762,  118.49308143, -109.97924008],
            dtype=float
        )

        assert self.model.nu == 18, f"Expected 18 actuators, found {self.model.nu}"

        self.jntCondition = np.ones(18, dtype=int)
        self.prev_error = np.zeros(18)

        # PD gains
        self.kp = 250.0
        self.kd = 100.0

        self.body_pos = []
        self.body_quat = []
        self.euler_angles=[]
        self.pos = np.zeros(3, dtype=float)

        # For velocity tracking
        self.prev_yaw = None

        # actuator and joint mapping
        self.act_start = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "hip_joint_1")
        self.joint_start = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "hip_joint_1")

    def render(self):
        if self.viewer is None and self.render_mode:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.distance = 7.0

        if self.viewer is not None:
            self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def checkingLegCondition(self):
        self.jntCondition[:] = 1
        for leg in range(self.locomotion.NumberOfLeg):
            if self.locomotion.legCondition[leg] == 0:
                for i in range(3):
                    self.jntCondition[leg * 3 + i] = 0

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = [0.0, 0.0, self.initial_height]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[7:25] = self.initial_qpos
        self.data.qvel[:] = 0.0

        print("INFO: initializing locomotion ...")
        mujoco.mj_forward(self.model, self.data)

        # stabilize a bit
        for _ in range(20):
            self.data.qpos[2] = self.initial_height
            self.data.ctrl[:] = self.init_ctrl
            mujoco.mj_step(self.model, self.data)
            if self.render_mode:
                self.render()

        # reset locomotion internal state
        self.locomotion.__init__()

        # set gear + initial ctrl
        for id in range(self.act_start, 18):
            self.locomotion.gear[id] = 100
            a = self.locomotion.angle_to_pos(id, self.locomotion.theta[id])
            self.data.ctrl[id] = a * self.locomotion.gear[id]

        self.prev_error[:] = 0.0
        self.prev_yaw = None
        self.pos[:] = [self.data.qpos[0], self.data.qpos[1], 0.0]

        if self.render_mode:
            self.render()

    def _get_terminated(self):
        base_height = self.data.qpos[2]
        if base_height < (self.initial_height - 0.9) or base_height > (self.initial_height + 0.9):
            print(f"Base height out of range: {base_height:.3f}")

        world_quat = self.data.qpos[3:7]
        euler_angles = R.from_quat([world_quat[1], world_quat[2], world_quat[3], world_quat[0]]).as_euler(
            'xyz', degrees=True
        )

        roll = euler_angles[0]
        pitch = euler_angles[1]

        if abs(roll) > 70 or abs(pitch) > 70:
            print(f"Termination: tilt too large (roll={roll:.2f}, pitch={pitch:.2f})")
            return True

        return False

    def apply_pd_control(self):
        dt = self.model.opt.timestep

        for id in range(self.act_start, 18):
            joint_index = id + (self.joint_start - self.act_start)
            qpos_index = self.model.jnt_qposadr[joint_index]
            qvel_index = self.model.jnt_dofadr[joint_index]

            target_pos = self.locomotion.angle_to_pos(id, self.locomotion.theta[id])
            current_pos = self.data.qpos[qpos_index]
            current_vel = self.data.qvel[qvel_index]

            error = target_pos - current_pos
            d_error = error - self.prev_error[id]

            if self.jntCondition[id] == 1:
                self.data.ctrl[id] = self.kp * error + self.kd * d_error
            else:
                self.data.ctrl[id] = 0.0

            self.prev_error[id] = error

    def run(self,
            TAU=0.1,
            lstep_x=100,
            lstep_y=40,
            swing_time=10,
            angle_speed=1,
            outer_loops=1000,
            inner_loops=500,
            constant_raw=None):

        if constant_raw is None:
            constant_raw = np.zeros(12)

        gain_raw = np.zeros(12)

        # oscillator parameters
        self.oscillator.inPolicyConstant = constant_raw.copy()
        self.oscillator.inPolicyGain = gain_raw.copy()

        paramsLoco = np.array([lstep_x, lstep_y, swing_time, angle_speed])
        paramsOsc = np.concatenate([[TAU], constant_raw, gain_raw])

        # enable all legs
        for i in range(self.locomotion.NumberOfLeg):
            self.locomotion.legCondition[i] = 1

        print("----- Simulation Start -----")
        print("paramsLoco =", paramsLoco)
        print("paramsOsc =", paramsOsc)

        for step_idx in range(outer_loops):
            
            lstep = min(abs(lstep_x + lstep_y), 100)
            TAU = 0.15 + 0.1 * (100 - lstep) / 100
            swing_time = 6 + 10 * (100 - lstep) / 100
            
            paramsLoco = np.array([lstep_x, lstep_y, swing_time, angle_speed])
            paramsOsc = np.concatenate([[TAU], constant_raw, gain_raw])

            
            if self.locomotion.count == 0:
                self.locomotion.legPos = -self.locomotion.legPos

            self.locomotion.count += 1

            if self.locomotion.count > 1:
                self.checkingLegCondition()
                self.locomotion.motionGenerator(paramsLoco, paramsOsc)

            # inner physics loop
            for _ in range(inner_loops):
                self.locomotion.inverse_kinematics()
                self.apply_pd_control()
                mujoco.mj_step(self.model, self.data)

            # print status
            self.body_pos = self.data.qpos[0:3]
            self.body_quat = self.data.qpos[3:7]

            self.euler_angles = R.from_quat(
                [self.body_quat[1], self.body_quat[2], self.body_quat[3], self.body_quat[0]]
            ).as_euler('xyz', degrees=True)
            self.pos = np.array(
                [
                    self.body_pos[0],
                    self.body_pos[1],
                    model_yaw_to_robot_heading_deg(self.euler_angles[2]),
                ],
                dtype=float
            )

            #print(f"[{step_idx}] Pos=({self.body_pos[0]:.3f}, {self.body_pos[1]:.3f}, {self.body_pos[2]:.3f}) "f"Euler=({self.euler_angles[0]:.2f}, {self.euler_angles[1]:.2f}, {self.euler_angles[2]:.2f})")

            if self.render_mode:
                self.render()

            if self._get_terminated():
                print("Simulation terminated.")
                break
            #print(f"Final pos = [{self.pos[0]:.3f}, {self.pos[1]:.3f}, {self.pos[2]:.2f}]")

        return self.pos.copy()


if __name__ == "__main__":
    

    sim = SingleLegRobotSim(render_mode=True)
    sim.reset()
    pos = sim.run(
        TAU=0.15, # to control frequency of gait
        lstep_x=100, # forward/backward stride command: to control step length in x or front direction -100 to 100
        lstep_y=0, # to control step length in y or side direction -100 to 100
        swing_time=6, # to control speed time of swinging leg
        angle_speed=0.02, #  yaw/turning increment: to control rotation speed -0.02 to 0.02
        outer_loops=10000,
        inner_loops=500
    )
    
    print(f"Final pos = [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.2f}]")
    sim.close()
