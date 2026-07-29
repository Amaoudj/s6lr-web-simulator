import numpy as np
import time

class Oscillator:
    def __init__(self, num_oscillators=6, num_legs=6, dt=0.05, t_max=100.0, tau=0.3, alpha=0.1, e=1.0):
        # Constants
        self.NUM_OSCILLATORS = num_oscillators
        self.NUM_LEG = num_legs
        self.DT = dt
        self.T_MAX = t_max
        self.TAU = tau
        self.ALPHA = alpha
        self.EE = e
                
        self.prev_v = np.zeros(self.NUM_OSCILLATORS)  # Track x from previous timestep
        self.prev_x = np.zeros(self.NUM_OSCILLATORS)  # Track x from previous timestep
        self.Imp_x = np.zeros(self.NUM_OSCILLATORS)  # Impulse output (0 or 1)

        # ID array
#        self.id = np.array([4, 0, 1, 5, 2, 3], dtype=int)
        self.id = np.array([0,1,2,3,5,4], dtype=int)

        # Coupling weights (adjustable later via failure logic)
        A = 1.0  # N2 fail → A↑ ; N3 fail → A↓
        B = 1.0  # N2 fail → B↓ ; N3 fail → B↑
        C = 1.0  # N4 fail → C↑ ; N5 fail → C↓
        D = 1.0  # N4 fail → D↑ ; N5 fail → D↓
        E = 1.0  # N0 fail → E↑ ; N1 fail → E↓
        F = 0.0  # N0 fail → F↓ ; N1 fail → F↑
        G = 1.0  # Used in multiple couplings
        H = 0.0
        I = 0.0
        J = 0.0
        K = 0.0
        L = 0.0
        
        # Coupling matrices
        self.a = np.array([
            [0, 1, 1, 1, 0, 0],
            [1, 0, 1, 1, 0, 0],
            [1, 1, 0, 1, 1, 1],
            [1, 1, 1, 0, 1, 1],
            [0, 0, 1, 1, 0, 1],
            [0, 0, 1, 1, 1, 0],
        ], dtype=float) * 1

        self.b = np.array([
            [0, G, C, H, A, L],
            [G, 0, H, D, L, B],
            [C, I, 0, G, E, J],
            [I, D, G, 0, J, F],
            [A, L, E, K, 0, G],
            [L, B, K, F, G, 0],
        ], dtype=float) * -0.5

        self.c = np.array([
            [0, 1,  0,  0,  0,  0],
            [1, 0,  0,  0,  0,  0],
            [0, 0,  0,  1,  0,  0],
            [0, 0,  1,  0,  0,  0],
            [0, 0,  0,  0,  0,  1],
            [0, 0,  0,  0,  1,  0],
        ], dtype=float) * 0.0

        # Malfunction coupling matrix for pattern adaptation
        # This parameter is for changing the pattern from tripedal balance locomotion to front mid locomotion
        self.c_malfunction = np.array([
            # Leg 0
            [[0,   -1,    0,    0,    0,    1],
             [-1,   0,    1,    0,    0,    0],
             [0,    1,    0,   -1,   -1,    0],
             [0,    0,   -1,    0,    1,    1],
             [0,    0,   -1,    1,    0,   -1],
             [1,    0,    0,    1,   -1,    0]],
            # Leg 1
            [[0,   -1,    0,    0,    0,    1],
             [-1,   0,    1,    0,    0,    0],
             [0,    1,    0,   -1,   -1,    0],
             [0,    0,   -1,    0,    1,    1],
             [0,    0,   -1,    1,    0,   -1],
             [1,    0,    0,    1,   -1,    0]],
            # Leg 2
            [[0,   -1,    0,    0,    0,    1],
             [-1,   0,    1,    0,    0,    0],
             [0,    1,    0,   -1,   -1,    0],
             [0,    0,   -1,    0,    1,    1],
             [0,    0,   -1,    1,    0,   -1],
             [1,    0,    0,    1,   -1,    0]],
            # Leg 3
            [[0,   -1,    0,    0,    0,    1],
             [-1,   0,    1,    0,    0,    0],
             [0,    1,    0,   -1,   -1,    0],
             [0,    0,   -1,    0,    1,    1],
             [0,    0,   -1,    1,    0,   -1],
             [1,    0,    0,    1,   -1,    0]],
            # Leg 4
            [[0,   -1,    0,    0,    0,    1],
             [-1,   0,    1,    0,    0,    0],
             [0,    1,    0,   -1,   -1,    0],
             [0,    0,   -1,    0,    1,    1],
             [0,    0,   -1,    1,    0,   -1],
             [1,    0,    0,    1,   -1,    0]],
            # Leg 5
            [[0,   -1,    0,    0,    0,    1],
             [-1,   0,    1,    0,    0,    0],
             [0,    1,    0,   -1,   -1,    0],
             [0,    0,   -1,    0,    1,    1],
             [0,    0,   -1,    1,    0,   -1],
             [1,    0,    0,    1,   -1,    0]]
        ], dtype=float)

        # Network matrix (calculated dynamically)
        self.net = np.zeros((self.NUM_OSCILLATORS, self.NUM_OSCILLATORS), dtype=float)

        self.a *= 0  # disable a-matrix

        # States
        self.s = np.zeros(self.NUM_OSCILLATORS)
        self.v = np.full(self.NUM_OSCILLATORS, 0.5)
        self.v0 = np.full(self.NUM_OSCILLATORS, 0.5)
        self.x = np.array([3, 2, 1, 1, 1, 1], dtype=float)
        self.x_condition = np.array([0, 1, 1, 1, 1, 1], dtype=int)
        self.F = np.zeros(self.NUM_OSCILLATORS)
        self.sw_time = np.zeros(self.NUM_OSCILLATORS)
        self.sw = np.zeros(self.NUM_OSCILLATORS)
                
        # Initialize with warmup
        print("init\n")
        
        for i in range(1000):
            self.weight_calculation()
            self.runge_kutta_step(self.TAU)
            self.detect_impulse(0.0)
                    
#            exit()
            
        print("end\n")
#        exit()
#        exit()

    def weight_calculation(self):
        """Calculate network weights based on leg conditions"""
        for i in range(self.NUM_OSCILLATORS):
            for j in range(self.NUM_OSCILLATORS):
                self.net[i, j] = self.b[i, j]
                for k in range(self.NUM_OSCILLATORS):
                    self.net[i, j] += self.c_malfunction[k, i, j] * 0.2 * (self.x_condition[self.id[k]] - 1)
        
        # Print the network matrix
#        print("Network matrix (net):")
#        print(self.net)

    def compute_v_dot(self, i, v, x, TAU):
        """Compute dv_i/dt using the net matrix"""
        coupling = 0.0
        for j in range(self.NUM_OSCILLATORS):
            coupling += self.net[self.id[i], self.id[j]] * v[j] + self.c[i, j] * self.sw[j]
        
        
        r2 = x[i] ** 2 + v[i] ** 2
#        print(r2)
#        if self.x_condition[i] == 1:
        return (1.0 / TAU) * (-self.ALPHA * (r2 - self.EE) * v[i] / self.EE - x[i] + coupling)
#        else:
#            return 0.0

    def compute_x_dot(self, i, v, TAU):
#        if self.x_condition[i] == 1:
        return v[i] / TAU
#        else:
#        return 0.0

    def calculate_spike(self, thresh, v, v0):
        return 1.0 if v > thresh and v0 <= thresh else 0.0

    def set_coupling_matrix(self, new_b):
        self.b = new_b.copy()
        print("Update Coupling Matrix (b):")
        for row in self.b:
            print("  ", " ".join(f"{val:.2f}" for val in row))
        
    def runge_kutta_step(self, TAU):
#    """Runge-Kutta 4th order integration - matches C implementation"""
    
        # Print velocities
#        print(self.v)
        
        # Initialize k arrays
        k1x = np.zeros(self.NUM_OSCILLATORS)
        k2x = np.zeros(self.NUM_OSCILLATORS)
        k3x = np.zeros(self.NUM_OSCILLATORS)
        k4x = np.zeros(self.NUM_OSCILLATORS)
        
        k1v = np.zeros(self.NUM_OSCILLATORS)
        k2v = np.zeros(self.NUM_OSCILLATORS)
        k3v = np.zeros(self.NUM_OSCILLATORS)
        k4v = np.zeros(self.NUM_OSCILLATORS)
        
        x_temp = np.zeros(self.NUM_OSCILLATORS)
        v_temp = np.zeros(self.NUM_OSCILLATORS)
        
        # Buffer arrays to store original values
        v_buff = np.zeros(self.NUM_OSCILLATORS)
        x_buff = np.zeros(self.NUM_OSCILLATORS)
        
        # Store original values in buffer
        for i in range(self.NUM_OSCILLATORS):
            v_buff[i] = self.v[i]
            x_buff[i] = self.x[i]
                                                        
#        print(f"{v_buff}\t", end="\n")
#        print(f"{x_buff}\t", end="\n")
        # Compute k1 for all oscillators
        for i in range(self.NUM_OSCILLATORS):
            k1x[i] = self.DT * self.compute_x_dot(i, self.v, TAU)
            k1v[i] = self.DT * self.compute_v_dot(i, self.v, self.x, TAU)
            
            # Compute temp values for k2
            x_temp[i] = self.x[i] + 0.5 * k1x[i]
            v_temp[i] = self.v[i] + 0.5 * k1v[i]
                                                                                                           
#        print(f"{k1x}\t", end="\n")
#        print(f"{k1v}\t", end="\n")
#        print(f"{v_temp}\t", end="\n")
        # Compute k2 for all oscillators
        for i in range(self.NUM_OSCILLATORS):
            k2x[i] = self.DT * self.compute_x_dot(i, v_temp, TAU)
            k2v[i] = self.DT * self.compute_v_dot(i, v_temp, x_temp, TAU)
            
            # Compute temp values for k3
            x_temp[i] = self.x[i] + 0.5 * k2x[i]
            v_temp[i] = self.v[i] + 0.5 * k2v[i]
                                    
#        print(f"{v_temp}\t", end="")
        # Compute k3 for all oscillators
        for i in range(self.NUM_OSCILLATORS):
            k3x[i] = self.DT * self.compute_x_dot(i, v_temp, TAU)
            k3v[i] = self.DT * self.compute_v_dot(i, v_temp, x_temp, TAU)
            
            # Compute temp values for k4
            x_temp[i] = self.x[i] + k3x[i]
            v_temp[i] = self.v[i] + k3v[i]
                                    
#        print(f"{v_temp}\t", end="")
        # Compute k4 for all oscillators
        for i in range(self.NUM_OSCILLATORS):
            k4x[i] = self.DT * self.compute_x_dot(i, v_temp, TAU)
            k4v[i] = self.DT * self.compute_v_dot(i, v_temp, x_temp, TAU)
        
        # Update states using RK4 formula
        for i in range(self.NUM_OSCILLATORS):
            self.x[i] += (k1x[i] + 2 * k2x[i] + 2 * k3x[i] + k4x[i]) / 6.0
            self.v[i] += (k1v[i] + 2 * k2v[i] + 2 * k3v[i] + k4v[i]) / 6.0
        
            
#        print(self.v)

        for i in range(self.NUM_OSCILLATORS):
            self.F[i] = self.calculate_spike(0.0, self.v[i], self.v0[i])
            
            if self.F[i] > 0:
                self.sw_time[i] = 20
                if i == 0:
                    self.sw_time[i] = 100
            
            if self.sw_time[i] > 0:
                self.sw[i] = -2
                self.sw_time[i] -= 1
            else:
                self.sw[i] = 0
            
            # Override: set sw[i] to 0 (as in C code)
            self.sw[i] = 0
            
            self.v0[i] = self.v[i]
            
    def detect_impulse(self, threshold=0.0):
#        print(self.v)
        for i in range(self.NUM_OSCILLATORS):
            if self.prev_v[i] < threshold and self.v[i] >= threshold:
                self.Imp_x[i] = 1.0  # Rising edge detected
            else:
                self.Imp_x[i] = 0.0  # No impulse
            # If oscillator is disabled, no impulse
            if self.x_condition[i] == 0:
                self.Imp_x[i] = 0.0
            self.prev_v[i] = self.v[i]  # Update previous x
            self.prev_x[i] = self.x[i]  # Update previous x
                                        
#        print(f"{self.Imp_x}\t", end="\t")
#        print(f"{self.x_condition}\t", end="")
    def simulate(self, print_values=True, delay=0.01):
        print("Starting simulation...")
        t = 0.0
        while t < self.T_MAX:
            self.weight_calculation()  # Recalculate weights based on leg conditions
            self.runge_kutta_step(self.TAU)
            
            if print_values:
                print(f"Time {t:.2f}s:")
                for i in range(self.NUM_OSCILLATORS):
                    print(f"{self.x[i]:.3f}\t", end="")
                print()
            t += self.DT
            time.sleep(delay)

# Usage
if __name__ == "__main__":
    osc = Oscillator()
    osc.simulate()
