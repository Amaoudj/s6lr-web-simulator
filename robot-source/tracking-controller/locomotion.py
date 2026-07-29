import numpy as np
import csv

from oscillator import Oscillator

class Locomotion:
    PI = 3.14159
    DIM = 3
    NumberOfLeg = 6
    
    def __init__(self):
                        
        self.oscillator = Oscillator()
        self.torsoCoM = np.zeros(self.DIM)
        self.torsoInitCoM = np.zeros(self.DIM)
        self.torso_t = np.zeros(self.DIM)
        self.torso_rot = 0
        self.torso_speed = np.zeros(self.DIM)
                
        self.legCondition = np.ones(self.NumberOfLeg, dtype=int)  # 1 = active,
        self.legTouchSensor = np.ones(self.NumberOfLeg, dtype=int)  # 1 = active,
        
        self.theta = np.zeros(20)
        H_base = -40
        self.baseLeg = np.array([
            [50.0, 24.5, -22],
            [50.0, -24.5, -22],
            [0, 28.0, -22],
            [0, -28.0, -22],
            [-50.0, 24.5, -22],
            [-50.0, -24.5, -22]
        ])
        self.initLegPos = np.array([
            [50, 60, H_base],
            [50, -60, H_base],
            [0, 70, H_base],
            [0, -70, H_base],
            [-50, 60, H_base],
            [-50, -60, H_base]
        ])
#        self.initLegPosRun = np.array([
#            [50, 60, H_base],
#            [80, -40, H_base],
#            [30, 70, H_base],
#            [40, -70, H_base],
#            [-10, 60, H_base],
#            [-50, -60, H_base]
#        ])
        self.initLegPosRun = np.array([
            [50, 60, H_base],
            [50, -60, H_base],
            [0, 70, H_base],
            [0, -70, H_base],
            [-50, 60, H_base],
            [-50, -60, H_base]
        ])
        self.legAdj_conn = np.array([
            [0,     -0.0,     2.0,    0,      0,      0],
            [0.0,     0,      0,      -2.0,   0,      0],
            [-1,    -0.5,   0,      0,      1,      0.5],
            [0.5,   1,      0,      0,      -0.5,   1],
            [0,     0,      -1.5,   0,      0,      1],
            [0,     0,      0,      1.5,    -1,      0],
        ], dtype=float)
        
        
        self.addInitLeg = np.zeros((6, 3))
        self.initLeg_adj = np.zeros((6, 3))
                                
        self.hipAngle_adj = np.zeros(6)
        self.hipAngle_adj_prev = np.zeros(6)
        
        self.legPos = np.zeros((6, 3))
        self.initAngle = np.array([
            0, -1.274, 2.782, 0, 1.274, -2.782,
            0, -1.274, 2.782, 0, 1.274, -2.782,
            0, -1.274, 2.782, 0, 1.274, -2.782,
            0, 0
        ])
        self.dir = np.array([
            1, 1, -1, -1, -1, 1,
            1, 1, -1, -1, -1, 1,
            1, 1, -1, -1, -1, 1,
            1, 1
        ])
        
        #optimized paramters
        self.rsl_time = 1000;
        self.torso_speed = np.array([0.0, 0.0, 0.0])
        
        # Number of legs
        self.NumberOfLeg = 6

        # Torso center of mass and translation
        self.torsoCoM = np.array([0.0, 0.0, 0.0])
        self.torso_t = np.array([0.0, 0.0, 0.0])  # Assuming DIM = 3

        self.v_max = 2
        self.acc_max = 0.2
        self.torso = np.zeros(self.DIM)
        self.v_torso = np.zeros(self.DIM)
        self.acc_torso = np.zeros(self.DIM)
        self.torsoRot = np.zeros(self.DIM)
        self.v_torso_rot = 0
        self.actSTEP = np.zeros(self.NumberOfLeg, dtype=int)
        self.legSTATE = np.zeros(self.NumberOfLeg, dtype=int)
        self.changePosture = np.zeros(self.NumberOfLeg, dtype=int)
        self.legStep = np.zeros((6, 3))
        self.legStep0 = np.zeros((6, 3))
        self.legStepRot = np.zeros((6, 3))
        self.legStepRot0 = np.zeros((6, 3))
        self.step_t = np.zeros((6, 3))
        self.step_h = np.zeros((6, 3))
        self.stepTime = np.zeros(6)
        self.cTime = np.zeros(6)
        self.timeInc = np.zeros(6)
        self.forceSensors = np.zeros(6)
        self.supports = 6
        self.orderStep = np.zeros(6, dtype=int)
        self.v_LegRot = np.zeros((6, 3))
        self.vTheta = 0
        self.vTheta0 = 0
        self.nextStep = 0
        self.time_ = 0
        self.count = 0
        self.gear = np.zeros(18)
        self.thetaIK = np.zeros(20)
                
        self.L1 = 56.0
        self.L2 = 75.0
        self.L3 = 24.0
        
        self.order0 = [0, 3, 4, 1, 2, 5]
        self.order1 = [3, 4, 1, 2, 5, 0]
        self.b = [1.0,1.0,1.0,1.0,1.0,1.0]
        
                
        self.strechError = np.zeros(6)
        self.strechLength = np.zeros(6)
        
        self.csv_file = open("log_output.csv", mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        # Write header
        header = ["Step"] + \
                 [f"X{i}" for i in range(len(self.oscillator.v))] + \
                 [f"Imp_x{i}" for i in range(len(self.oscillator.Imp_x))] + \
                 [f"LegPos_{leg}_{axis}" for leg in range(6) for axis in ['x', 'y', 'z']]
        self.csv_writer.writerow(header)
        self.step_count = 0
        
    def angleToPos(self, id, angle):
        return self.initAngle[id] + self.dir[id] * angle
    
    
    def torsoMovement(self, dim, torso0, torso_t, v_torso0):
        devider = 2 + (self.v_max / self.acc_max)
        dt = torso_t - torso0
        a = dt / devider
        a = np.clip(a, -self.v_max, self.v_max)
        b = a - v_torso0
        b = np.clip(b, -self.acc_max, self.acc_max)
        self.v_torso[dim] += b
        return torso0 + self.v_torso[dim]
    
    def torsoMovements3D(self):
        for i in range(self.DIM):
            self.torso[i] = self.torsoMovement(i, self.torsoCoM[i], self.torso_t[i], self.v_torso[i])
    
    def torsoRotation(self):
        self.v_torso_rot = 0.04
    
    def getCoMvalue(self):
        active_legs = np.where(self.legCondition == 1)[0]  # Get indices of active legs

        if len(active_legs) == 0:
            self.torsoCoM = np.zeros(self.DIM)  # fallback: no active legs
        else:
        
            # Combine positions: legPos + initLegPos + baseLeg
#            combined_pos = self.legPos + self.initLegPos + self.baseLeg
            combined_pos = self.legPos
            self.torsoCoM = -np.sum(combined_pos[active_legs], axis=0) / len(active_legs)
        self.torsoCoM[2] = 0
#        print(self.legPos[:, :2])
#        print("Torso CoM:", self.torsoCoM)
        
    def generateStepActivation(self):
        supports = self.NumberOfLeg - np.sum(self.legSTATE)
        minSupport = self.NumberOfLeg // 2 + (1 if self.NumberOfLeg % 2 != 0 else 0)

        if self.nextStep == 3:
            if all(self.legSTATE[self.order0[i]] == 0 for i in range(3)):
                supports = 4
            else:
                supports = 0
        elif self.nextStep == 0:
            if all(self.legSTATE[self.order0[i]] == 0 for i in range(3, 6)):
                supports = 4
            else:
                supports = 0

        if supports > minSupport:
            # Compute leg-to-torso distances
            legTorsoDis = np.linalg.norm(self.legPos[:, :2], axis=1)

            # Compute torso position distances
            torsoRes = np.linalg.norm(self.torsoCoM[:2])
            torso_t_Res = np.linalg.norm(self.torso_t[:2])

            # Determine next step
            leg = self.order0[self.nextStep]
            limitDistance = 2  # + (6 - supports) * 3

            if legTorsoDis[leg] > limitDistance and self.legSTATE[leg] == 0 and torsoRes >= torso_t_Res / 6:
                self.actSTEP[leg] = 1
                self.nextStep += 1
                if self.nextStep >= self.NumberOfLeg:
                    self.nextStep = 0
                    
    def checkingSteppingState(self):
        print(f"{self.actSTEP}\t", end="")
        for leg in range(self.NumberOfLeg):
            if self.actSTEP[leg] == 1:
                self.actSTEP[leg] = 0
                self.legSTATE[leg] = 1
                self.step_t[leg][0] = self.torso_t[0] / 2 - self.legPos[leg][0]
                self.step_t[leg][1] = self.torso_t[1] / 2 - self.legPos[leg][1]
                self.cTime[leg] = 0
                self.stepTime[leg] = self.rsl_time;
                self.step_h[leg][2] = 80
                if self.changePosture[leg] == 1:
                    for i in range(3):
                        self.legPos[leg][i] -= self.initLegPos[leg][i] - self.initLegPosRun[leg][i]
                        self.initLegPosRun[leg][i] = self.initLegPos[leg][i] + self.initLeg_adj[leg][i]
                    self.changePosture[leg] = 0



#            elif self.cTime[leg] > 0.8 * self.stepTime[leg] and self.legSTATE[leg] >= 1 and self.legTouchSensor[leg] == 1:
            elif self.cTime[leg] > 1 * self.stepTime[leg] and self.legSTATE[leg] >= 1:
                self.legSTATE[leg] = 0
                self.legStep[leg] = np.zeros(self.DIM)
            elif self.cTime[leg] > self.stepTime[leg] and self.legSTATE[leg] == 1 and self.legTouchSensor[leg] == 0:
                self.legSTATE[leg] = 2
                
    def rotation_z(self, a, pos):
        """Rotate a 3D position `pos` around the Z-axis by angle `a` (in radians)."""
        pos = np.asarray(pos)
        pos1 = np.zeros(3)
        
        pos1[0] = pos[0] * np.cos(a) - pos[1] * np.sin(a)
        pos1[1] = pos[0] * np.sin(a) + pos[1] * np.cos(a)
        pos1[2] = pos[2]  # Z stays the same
        
        return pos1


    def getTargetPosFromRotation(self, dTheta):
        """Compute v_LegRot from rotated position by angle `dTheta`."""

        for leg in range(self.NumberOfLeg):
            # Combine positions: legPos + initLegPosRun + baseLeg
            pos = self.legPos[leg] + self.initLegPosRun[leg] + self.baseLeg[leg]
            
            # Apply rotation
            pos1 = self.rotation_z(dTheta, pos)
            
            # Compute difference as rotational shift
            self.v_LegRot[leg] = pos1 - pos
            
        
    def motionGenerator(self, paramsLoco, paramsOsc):
        
        
#        self.adjustInitLeg()
                                
#        print(f"{self.legCondition}\t", end="")
#        print(f"{self.oscillator.x_condition}\t", end="")
#        print(f"{self.legStep[0]}\t", end="")
        self.oscillator.x_condition = self.legCondition.copy()
#        self.oscillator.runge_kutta_step(paramsOsc)
        self.oscillator.runge_kutta_step(paramsOsc[0])
        self.oscillator.detect_impulse()
    
        
        for i in range(self.oscillator.NUM_OSCILLATORS):
#            print(f"{self.oscillator.Imp_x[i]:.3f}\t", end="")
            self.actSTEP[i] = int(self.oscillator.Imp_x[i])
#        print("actSTEP:", "\t".join(str(val) for val in self.actSTEP[:self.oscillator.NUM_OSCILLATORS // 2]))
#        print("m_neuron:", "\t".join(str(val) for val in self.oscillator.m[:self.oscillator.NUM_OSCILLATORS // 2]))
#        print()
                
                
#        self.legCondition[4] = 0
        
        self.torso_speed[0] = paramsLoco[0];
        self.torso_speed[1] = paramsLoco[1];
                
        self.torso_t[0] = self.torso_speed[0]
        self.torso_t[1] = self.torso_speed[1]
        
        self.rsl_time = paramsLoco[2];
        self.torso_rot = paramsLoco[3];
        
        
        if False and self.time_ > 5.000:
            self.torso_t[0] = self.torso_speed[0]
            self.torso_t[1] = self.torso_speed[1]
            self.torso_rot = 0.00
#            self.hipAngle_adj[0] = 30
#            self.hipAngle_adj[1] = 30
        
#            self.legCondition[0] = 0
#            self.legCondition[2] = 1
#            self.legCondition[4] = 1
            
        self.time_ += 0.01
        
#        self.generateStepActivation()
        self.checkingSteppingState()
        
        self.getCoMvalue()
        self.torsoMovements3D()
        
        self.vTheta0 = self.vTheta
#        self.vTheta = 0.4 * np.sin(self.time_)
        self.vTheta += self.torso_rot
        
        for leg in range(self.NumberOfLeg):
            
            v_step0 = self.legStep[leg] - self.legStep0[leg]
            self.legStep0[leg] = self.legStep[leg].copy()  # Store previous step
            
            if self.legSTATE[leg] == 0:
                self.legStep[leg] = np.zeros(self.DIM)  # Reset step if leg is grounded
                self.legStep0[leg] = np.zeros(self.DIM)
                # The vertical swing arc is a transient foot-lift command, not
                # a permanent foot placement offset. The old slow decay caused
                # legPos[:, 2] to drift upward over repeated steps until the
                # robot lost support and the chassis touched the floor.
                self.legPos[leg][2] = 0.0
            elif self.legSTATE[leg] == 1:
                self.timeInc[leg] = self.cTime[leg] / self.stepTime[leg]
                self.cTime[leg] += 1
                
                self.legStep[leg] = (self.step_t[leg] * ((2 - (np.cos(np.pi * self.timeInc[leg]) + 1)) * 0.5) + (np.sin(np.pi * self.timeInc[leg])) * self.step_h[leg])
            elif self.legSTATE[leg] == 2:
                self.legStep[leg][2] += v_step0[2]
                    
#            self.oscillator.sw[leg] = self.legSTATE[leg] * -5
                            
#            if leg == 1:
#                print(f"STATE: {self.legSTATE[leg]}, Step Z: {self.legStep[leg][2]:.2f},\t{self.legPos[leg][2]:.2f},\t{self.forceSensors[leg]:.2f},\t{self.legTouchSensor[leg]}")
                        
        formatted_print = "\t".join(f"{f:.0f}" for f in self.hipAngle_adj)
#        print(f"{self.time_}\tSTATE: {self.legSTATE}\tangle Z: {formatted_print}")

#        print("sw =", "\t".join(f"{x}" for x in self.oscillator.sw))
        self.getTargetPosFromRotation(self.vTheta -  self.vTheta0)
        # Update leg positions
        for leg in range(self.NumberOfLeg):
            v_step = self.legStep[leg] - self.legStep0[leg]
            self.legPos[leg] += v_step - self.v_torso + self.v_LegRot[leg]
#            print("legPos[{}] = {}".format(leg, self.legPos[leg]))
        # print(f"{self.time_:.3f}\t{self.torsoCoM[0]:.3f}\t{self.v_torso[0]:.3f}\t{self.legPos[0][2]:.3f}", end="\t")
#        for leg in range(2):
#            print(f"{self.legPos[leg][1]:.3f}", end="\t")
        # print("\t".join(map(str, self.legSTATE)))
        
        # Logging data
        row = [self.step_count] + \
              list(self.oscillator.v) + \
              list(self.oscillator.Imp_x) + \
              [coord for leg in self.legPos for coord in leg]
        self.csv_writer.writerow(row)
        self.step_count += 1

    
    def angle_to_pos(self, id, angle):
        value = self.initAngle[id] + self.dir[id] * angle
        return value

    def inverse_kinematics_step2(self, x, y, leg):
        # Step 1: Calculate original leg vector and its length
        leg_vector = np.array([x, y])
        leg_length = np.linalg.norm(leg_vector)
        max_leg_length = self.L1 + self.L2
        
                
        self.strechLength[leg] = leg_length;
        self.strechError[leg] = 0;
        # Step 2: If overlength, rescale x and y
        if leg_length > max_leg_length:
#            print(f"⚠️  Warning: Leg {leg} overlength! Clipping from {leg_length:.2f} to {max_leg_length:.2f}")
            scale = max_leg_length / leg_length
            x *= scale
            y *= scale
            leg_length = max_leg_length  # Now updated
            self.strechError[leg] = 1;

        # Step 3: Perform IK computation as usual
        r = leg_length
        A = np.arccos((self.L1**2 + self.L2**2 - r**2) / (2 * self.L1 * self.L2))
        self.thetaIK[2] = np.pi - A
        B = np.arcsin(y / r)
        if x < 0:
            B = -(np.pi + B)
        C = np.arcsin((np.sin(A) * self.L2) / r)
        D = np.arcsin((np.sin(A) * self.L1) / r)
        C = np.pi - A - D
        self.thetaIK[1] = B + C
            

    def inverse_kinematics_step1(self, x, y, z, leg):
        self.thetaIK[0] = np.arctan2(z, x)
        r = np.sqrt(x**2 + z**2)
        X = r - self.L3
        self.inverse_kinematics_step2(X, y, leg)

    def ik_leg_pos(self, leg, x, y, z):
        if leg % 2 == 0:
            self.inverse_kinematics_step1(y, z, x, leg)
        else:
            self.inverse_kinematics_step1(-y, z, x, leg)

        self.theta[leg * 3] = self.thetaIK[0]
        self.theta[leg * 3 + 1] = self.thetaIK[1]
        self.theta[leg * 3 + 2] = self.thetaIK[2]

    def inverse_kinematics(self):
        for leg in range(6):
            self.ik_leg_pos(leg,
                            self.legPos[leg][0] + self.initLegPosRun[leg][0],
                            self.legPos[leg][1] + self.initLegPosRun[leg][1],
                            self.legPos[leg][2] + self.initLegPosRun[leg][2])
                        
    def adjustInitLeg(self):
        active_legs = np.where(self.legCondition == 1)[0]  # Get indices of active legs

        if len(active_legs) == 0:
            self.torsoInitCoM = np.zeros(self.DIM)
            return
        
        
        for leg in range(6):
            self.hipAngle_adj[leg] = 0;
            for i in range(6):
                self.hipAngle_adj[leg] += (1 - self.legCondition[i]) * self.legAdj_conn[leg][i] * 20
            if self.hipAngle_adj[leg] > 30:
                self.hipAngle_adj[leg] = 30
            elif self.hipAngle_adj[leg] < -30:
                self.hipAngle_adj[leg] = -30
                
            if self.hipAngle_adj[leg] != self.hipAngle_adj_prev[leg]:
                self.changePosture[leg] = 1

        self.initLeg_adj = self.rotate_each_leg_position_difference(self.hipAngle_adj)
        self.hipAngle_adj_prev = self.hipAngle_adj.copy()

        
        
        
        # Step 1: Compute initial CoM
        combined_pos = self.initLegPosRun + self.baseLeg * 2
        self.torsoInitCoM = -np.sum(combined_pos[active_legs], axis=0) / len(active_legs)

        # Step 2: Define tolerance (you can tune this)
        tolerance = 5.0  # mm or units, depending on your scale

        # Step 3: If torsoInitCoM is off-center, adjust initLegPos to balance it
#        self.initLegPos[0][2] = -20
#        self.initLegPos[1][2] = -20
#        self.initLegPos[2][2] = -20
#        self.initLegPos[3][2] = -20
#        self.initLegPos[4][2] = -20
#        self.initLegPos[5][2] = -20
        
        if np.linalg.norm(self.torsoInitCoM[:2]) > tolerance:
#            print(f"⚠️ Adjusting initLegPos to center torso (CoM XY = {self.torsoInitCoM[:2]})")

            # Shift initLegPos in x and y by -torsoInitCoM (only for active legs)
            mu = 0.1
            for leg in active_legs:
                self.initLegPosRun[leg][:2] += (mu * self.torsoInitCoM[:2]).astype(int)
                
        

    def rotate_each_leg_position_difference(self, angle_deg_list):
        """
        Rotate each (x, y) of self.initLegPos by its own angle from angle_deg_list,
        and return the DIFFERENCE between rotated and original positions.

        Args:
            angle_deg_list (list or np.array): list of 6 angles in degrees, one for each leg

        Returns:
            np.array: difference between rotated and original initLegPos
        """
        assert len(angle_deg_list) == self.initLegPos.shape[0], "Angle list must match number of legs"

        new_initLegPos = self.initLegPos.copy()

        for i in range(self.initLegPos.shape[0]):
            angle_rad = np.deg2rad(angle_deg_list[i])  # Convert each angle individually

            # Rotation matrix for this leg
            rotation_matrix = np.array([
                [np.cos(angle_rad), -np.sin(angle_rad)],
                [np.sin(angle_rad),  np.cos(angle_rad)]
            ])

            xy = self.initLegPos[i, 0:2]  # (x, y) of this leg
            rotated_xy = rotation_matrix @ xy
            new_initLegPos[i, 0:2] = rotated_xy

        # Calculate the DIFFERENCE
        difference = new_initLegPos - self.initLegPos

        return difference
