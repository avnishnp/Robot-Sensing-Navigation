import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import sqrtm

class UnscentedKalmanFilter:
    """
    Unscented Kalman Filter for LiDAR and Radar sensor fusion.
    This implementation tracks an object in 2D space with position, velocity, and yaw angle.
    
    State vector (7 dimensions):
    [px, py, v, yaw, yaw_rate, ax, ay]
    px, py: position
    v: velocity magnitude
    yaw: heading direction
    yaw_rate: rate of change of heading
    ax, ay: acceleration in x and y directions
    """
    
    def __init__(self, dt=0.1, std_lidar=None, std_radar=None, process_noise=None):
        # Time step
        self.dt = dt
        
        # State dimension
        self.n_x = 7
        
        # Augmented state dimension (adds process noise dimensions)
        self.n_aug = 9
        
        # Sigma points scaling parameter
        self.lambda_ = 3 - self.n_aug
        
        # Weights for sigma points
        self.weights = np.zeros(2 * self.n_aug + 1)
        self.weights[0] = self.lambda_ / (self.lambda_ + self.n_aug)
        for i in range(1, 2 * self.n_aug + 1):
            self.weights[i] = 0.5 / (self.lambda_ + self.n_aug)
        
        # Initial state estimate
        self.x = np.zeros(self.n_x)
        
        # Initial covariance matrix
        self.P = np.eye(self.n_x) * 1.0
        
        # Process noise standard deviations
        if process_noise is None:
            # Default process noise values (longitudinal acceleration and yaw acceleration)
            self.std_a = 2.0  # longitudinal acceleration noise
            self.std_yawdd = 0.5  # yaw acceleration noise
        else:
            self.std_a = process_noise[0]
            self.std_yawdd = process_noise[1]
        
        # Measurement noise for LiDAR (position x and position y)
        if std_lidar is None:
            self.std_laspx = 0.15  # meters
            self.std_laspy = 0.15  # meters
        else:
            self.std_laspx = std_lidar[0]
            self.std_laspy = std_lidar[1]
        
        # Measurement noise for Radar (range, angle, range rate)
        if std_radar is None:
            self.std_radr = 0.3      # meters
            self.std_radphi = 0.03   # radians
            self.std_radrd = 0.3     # meters/second
        else:
            self.std_radr = std_radar[0]
            self.std_radphi = std_radar[1]
            self.std_radrd = std_radar[2]
        
        # Lidar measurement noise covariance matrix
        self.R_lidar = np.diag([self.std_laspx**2, self.std_laspy**2])
        
        # Radar measurement noise covariance matrix
        self.R_radar = np.diag([self.std_radr**2, self.std_radphi**2, self.std_radrd**2])
        
        # Sigma points
        self.Xsig_pred = np.zeros((self.n_x, 2 * self.n_aug + 1))
        
        # For tracking NIS (Normalized Innovation Squared) values
        self.nis_lidar = []
        self.nis_radar = []

    def initialize(self, x0):
        """Initialize state with first measurement"""
        self.x = x0
    
    def generate_sigma_points(self):
        """Generate sigma points"""
        # Create augmented state and covariance
        x_aug = np.zeros(self.n_aug)
        x_aug[:self.n_x] = self.x
        
        P_aug = np.zeros((self.n_aug, self.n_aug))
        P_aug[:self.n_x, :self.n_x] = self.P
        P_aug[self.n_x:, self.n_x:] = np.diag([self.std_a**2, self.std_yawdd**2])
        
        # Square root of P_aug
        A = sqrtm((self.lambda_ + self.n_aug) * P_aug)
        
        # Generate sigma points
        Xsig_aug = np.zeros((self.n_aug, 2 * self.n_aug + 1))
        Xsig_aug[:, 0] = x_aug
        
        for i in range(self.n_aug):
            Xsig_aug[:, i+1] = x_aug + A[:, i]
            Xsig_aug[:, i+1+self.n_aug] = x_aug - A[:, i]
            
        return Xsig_aug
    
    def predict_sigma_points(self, Xsig_aug):
        """Predict sigma points"""
        for i in range(2 * self.n_aug + 1):
            # Extract values for better readability
            px = Xsig_aug[0, i]
            py = Xsig_aug[1, i]
            v = Xsig_aug[2, i]
            yaw = Xsig_aug[3, i]
            yawd = Xsig_aug[4, i]
            a_noise = Xsig_aug[self.n_x, i]
            yawdd_noise = Xsig_aug[self.n_x+1, i]
            
            # Avoid division by zero
            if abs(yawd) > 0.001:
                px_p = px + v/yawd * (np.sin(yaw + yawd * self.dt) - np.sin(yaw))
                py_p = py + v/yawd * (np.cos(yaw) - np.cos(yaw + yawd * self.dt))
            else:
                px_p = px + v * np.cos(yaw) * self.dt
                py_p = py + v * np.sin(yaw) * self.dt
            
            v_p = v
            yaw_p = yaw + yawd * self.dt
            yawd_p = yawd
            
            # Add noise
            px_p += 0.5 * self.dt**2 * np.cos(yaw) * a_noise
            py_p += 0.5 * self.dt**2 * np.sin(yaw) * a_noise
            v_p += self.dt * a_noise
            yaw_p += 0.5 * self.dt**2 * yawdd_noise
            yawd_p += self.dt * yawdd_noise
            
            # Write predicted sigma points
            self.Xsig_pred[0, i] = px_p
            self.Xsig_pred[1, i] = py_p
            self.Xsig_pred[2, i] = v_p
            self.Xsig_pred[3, i] = yaw_p
            self.Xsig_pred[4, i] = yawd_p
            
            # Additional state components (acceleration)
            self.Xsig_pred[5, i] = Xsig_aug[5, i]  # ax
            self.Xsig_pred[6, i] = Xsig_aug[6, i]  # ay
            
    def predict_mean_covariance(self):
        """Predict state mean and covariance from sigma points"""
        # Predict state mean
        self.x = np.zeros(self.n_x)
        for i in range(2 * self.n_aug + 1):
            self.x += self.weights[i] * self.Xsig_pred[:, i]
        
        # Predict state covariance
        self.P = np.zeros((self.n_x, self.n_x))
        for i in range(2 * self.n_aug + 1):
            diff = self.Xsig_pred[:, i] - self.x
            # Normalize the yaw angle to be between -π and π
            diff[3] = self.normalize_angle(diff[3])
            self.P += self.weights[i] * np.outer(diff, diff)
    
    def predict(self):
        """Complete prediction step"""
        Xsig_aug = self.generate_sigma_points()
        self.predict_sigma_points(Xsig_aug)
        self.predict_mean_covariance()
    
    def normalize_angle(self, angle):
        """Normalize angle to be between -π and π"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def update_lidar(self, z_lidar):
        """Update state using LiDAR measurement (position x, y)"""
        # Measurement dimension
        n_z = 2
        
        # Mean predicted measurement
        z_pred = np.zeros(n_z)
        
        # Predicted measurement sigma points matrix
        Zsig = np.zeros((n_z, 2 * self.n_aug + 1))
        
        # Transform sigma points into measurement space
        for i in range(2 * self.n_aug + 1):
            Zsig[0, i] = self.Xsig_pred[0, i]  # px
            Zsig[1, i] = self.Xsig_pred[1, i]  # py
            
            # Calculate mean predicted measurement
            z_pred += self.weights[i] * Zsig[:, i]
        
        # Measurement covariance matrix S and cross-correlation matrix T
        S = np.zeros((n_z, n_z))
        Tc = np.zeros((self.n_x, n_z))
        
        for i in range(2 * self.n_aug + 1):
            z_diff = Zsig[:, i] - z_pred
            x_diff = self.Xsig_pred[:, i] - self.x
            x_diff[3] = self.normalize_angle(x_diff[3])
            
            S += self.weights[i] * np.outer(z_diff, z_diff)
            Tc += self.weights[i] * np.outer(x_diff, z_diff)
        
        # Add measurement noise covariance
        S += self.R_lidar
        
        # Kalman gain
        K = Tc @ np.linalg.inv(S)
        
        # Update state
        z_diff = z_lidar - z_pred
        self.x += K @ z_diff
        
        # Update covariance
        self.P -= K @ S @ K.T
        
        # Calculate NIS (Normalized Innovation Squared)
        nis = z_diff.T @ np.linalg.inv(S) @ z_diff
        self.nis_lidar.append(nis)
        
        return nis
    
    def update_radar(self, z_radar):
        """Update state using Radar measurement (range, angle, range rate)"""
        # Measurement dimension
        n_z = 3
        
        # Mean predicted measurement
        z_pred = np.zeros(n_z)
        
        # Predicted measurement sigma points matrix
        Zsig = np.zeros((n_z, 2 * self.n_aug + 1))
        
        # Transform sigma points into measurement space
        for i in range(2 * self.n_aug + 1):
            px = self.Xsig_pred[0, i]
            py = self.Xsig_pred[1, i]
            v = self.Xsig_pred[2, i]
            yaw = self.Xsig_pred[3, i]
            
            # Range
            Zsig[0, i] = np.sqrt(px**2 + py**2)
            
            # Angle
            Zsig[1, i] = np.arctan2(py, px)
            
            # Range rate
            vx = v * np.cos(yaw)
            vy = v * np.sin(yaw)
            if Zsig[0, i] < 0.001:  # Avoid division by zero
                Zsig[2, i] = 0
            else:
                Zsig[2, i] = (px * vx + py * vy) / Zsig[0, i]
            
            # Calculate mean predicted measurement
            z_pred += self.weights[i] * Zsig[:, i]
        
        # Ensure angle is normalized
        z_pred[1] = self.normalize_angle(z_pred[1])
        
        # Measurement covariance matrix S and cross-correlation matrix T
        S = np.zeros((n_z, n_z))
        Tc = np.zeros((self.n_x, n_z))
        
        for i in range(2 * self.n_aug + 1):
            z_diff = Zsig[:, i] - z_pred
            z_diff[1] = self.normalize_angle(z_diff[1])
            
            x_diff = self.Xsig_pred[:, i] - self.x
            x_diff[3] = self.normalize_angle(x_diff[3])
            
            S += self.weights[i] * np.outer(z_diff, z_diff)
            Tc += self.weights[i] * np.outer(x_diff, z_diff)
        
        # Add measurement noise covariance
        S += self.R_radar
        
        # Kalman gain
        K = Tc @ np.linalg.inv(S)
        
        # Update state
        z_diff = z_radar - z_pred
        z_diff[1] = self.normalize_angle(z_diff[1])
        self.x += K @ z_diff
        
        # Update covariance
        self.P -= K @ S @ K.T
        
        # Calculate NIS (Normalized Innovation Squared)
        nis = z_diff.T @ np.linalg.inv(S) @ z_diff
        self.nis_radar.append(nis)
        
        return nis


def generate_test_data(dt, num_timesteps=200):
    """Generate test data for a vehicle moving in a curved trajectory"""
    # Ground truth trajectory parameters
    true_states = []
    
    # Initial state
    # [px, py, v, yaw, yaw_rate, ax, ay]
    state = np.array([0.0, 0.0, 5.0, 0.0, 0.1, 0.0, 0.0])
    
    for i in range(num_timesteps):
        # Store current state
        true_states.append(state.copy())
        
        # Update position based on velocity, heading, and heading rate
        px, py, v, yaw, yaw_rate = state[:5]
        
        if abs(yaw_rate) > 0.001:
            px += v/yaw_rate * (np.sin(yaw + yaw_rate * dt) - np.sin(yaw))
            py += v/yaw_rate * (np.cos(yaw) - np.cos(yaw + yaw_rate * dt))
        else:
            px += v * np.cos(yaw) * dt
            py += v * np.sin(yaw) * dt
        
        yaw += yaw_rate * dt
        
        # Normalize yaw angle
        while yaw > np.pi:
            yaw -= 2.0 * np.pi
        while yaw < -np.pi:
            yaw += 2.0 * np.pi
        
        # Update state
        state[0] = px
        state[1] = py
        state[3] = yaw
        
        # Add small random noise to velocity and yaw rate for realism
        state[2] += np.random.normal(0, 0.05)  # Velocity noise
        state[4] += np.random.normal(0, 0.005)  # Yaw rate noise
    
    return np.array(true_states)

def generate_measurements(true_states, std_lidar, std_radar):
    """Generate LiDAR and Radar measurements from ground truth states"""
    lidar_measurements = []
    radar_measurements = []
    
    for state in true_states:
        px, py, v, yaw = state[:4]
        
        # Lidar measures position (px, py)
        lidar_meas = np.array([
            px + np.random.normal(0, std_lidar[0]),
            py + np.random.normal(0, std_lidar[1])
        ])
        lidar_measurements.append(lidar_meas)
        
        # Radar measures range, bearing, and range rate
        range_val = np.sqrt(px**2 + py**2)
        bearing = np.arctan2(py, px)
        
        # Range rate (projection of velocity on the line of sight)
        vx = v * np.cos(yaw)
        vy = v * np.sin(yaw)
        range_rate = (px * vx + py * vy) / range_val if range_val > 0.001 else 0
        
        radar_meas = np.array([
            range_val + np.random.normal(0, std_radar[0]),
            bearing + np.random.normal(0, std_radar[1]),
            range_rate + np.random.normal(0, std_radar[2])
        ])
        radar_measurements.append(radar_meas)
    
    return np.array(lidar_measurements), np.array(radar_measurements)

def run_ukf_simulation(dt=0.1, n_steps=200, std_lidar=None, std_radar=None, process_noise=None):
    """Run a simulation of UKF with both LiDAR and Radar measurements"""
    # Set default noise parameters if not provided
    if std_lidar is None:
        std_lidar = [0.15, 0.15]  # position x, position y
    if std_radar is None:
        std_radar = [0.3, 0.03, 0.3]  # range, bearing, range rate
    if process_noise is None:
        process_noise = [2.0, 0.5]  # longitudinal acceleration noise, yaw acceleration noise
    
    # Generate ground truth and measurements
    print("Generating test data...")
    true_states = generate_test_data(dt, n_steps)
    lidar_measurements, radar_measurements = generate_measurements(true_states, std_lidar, std_radar)
    
    # Initialize UKF
    print("Initializing UKF...")
    ukf = UnscentedKalmanFilter(dt, std_lidar, std_radar, process_noise)
    
    # Initial state based on first measurement (simplified)
    x0 = np.zeros(7)
    x0[0] = lidar_measurements[0, 0]  # px
    x0[1] = lidar_measurements[0, 1]  # py
    x0[2] = 5.0  # initial velocity guess
    ukf.initialize(x0)
    
    # Track estimated states
    estimated_states = [ukf.x.copy()]
    
    # Simulate
    print("Running UKF simulation...")
    for i in range(1, n_steps):
        # Prediction step
        ukf.predict()
        
        # Update with LiDAR (every step)
        ukf.update_lidar(lidar_measurements[i])
        
        # Update with Radar (every step)
        ukf.update_radar(radar_measurements[i])
        
        # Store estimated state
        estimated_states.append(ukf.x.copy())
        
        if i % 50 == 0:
            print(f"Processed {i}/{n_steps} steps")
    
    return true_states, lidar_measurements, radar_measurements, np.array(estimated_states), ukf

def plot_results(true_states, lidar_measurements, radar_measurements, estimated_states, ukf):
    """Plot the results of the UKF simulation"""
    plt.figure(figsize=(15, 10))
    
    # Plot trajectory
    plt.subplot(2, 2, 1)
    
    # Plot estimated trajectory
    plt.plot(estimated_states[:, 0], estimated_states[:, 1], 'r--', label='UKF Estimate')
    
    # Plot ground truth
    plt.plot(true_states[:, 0], true_states[:, 1], 'b-', label='Ground Truth')
    
    # Plot LiDAR measurements
    plt.scatter(lidar_measurements[:, 0], lidar_measurements[:, 1], 
               c='g', s=10, alpha=0.4, label='LiDAR')
    
    # Plot Radar measurements (convert from polar to Cartesian)
    radar_cartesian = []
    for meas in radar_measurements:
        r, phi = meas[0], meas[1]
        px = r * np.cos(phi)
        py = r * np.sin(phi)
        radar_cartesian.append([px, py])
    radar_cartesian = np.array(radar_cartesian)
    
    plt.scatter(radar_cartesian[:, 0], radar_cartesian[:, 1], 
               c='m', s=10, alpha=0.4, label='Radar')
    
    plt.title('Object Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True)
    
    # Plot position error
    plt.subplot(2, 2, 2)
    position_error = np.sqrt((true_states[:, 0] - estimated_states[:, 0])**2 + 
                            (true_states[:, 1] - estimated_states[:, 1])**2)
    plt.plot(position_error)
    plt.title('Position Error')
    plt.xlabel('Time Step')
    plt.ylabel('Error (m)')
    plt.grid(True)
    
    # Plot NIS values for LiDAR
    plt.subplot(2, 2, 3)
    plt.plot(ukf.nis_lidar)
    plt.axhline(y=5.991, color='r', linestyle='--', label='95% Chi-Square')  # 95% for 2 DOF
    plt.title('NIS for LiDAR Measurements')
    plt.xlabel('Time Step')
    plt.ylabel('NIS')
    plt.legend()
    plt.grid(True)
    
    # Plot NIS values for Radar
    plt.subplot(2, 2, 4)
    plt.plot(ukf.nis_radar)
    plt.axhline(y=7.815, color='r', linestyle='--', label='95% Chi-Square')  # 95% for 3 DOF
    plt.title('NIS for Radar Measurements')
    plt.xlabel('Time Step')
    plt.ylabel('NIS')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def print_statistics(true_states, estimated_states, ukf):
    """Print statistics about the UKF performance"""
    # Calculate final errors
    final_pos_error = np.sqrt((true_states[-1, 0] - estimated_states[-1, 0])**2 + 
                             (true_states[-1, 1] - estimated_states[-1, 1])**2)
    final_vel_error = abs(true_states[-1, 2] - estimated_states[-1, 2])
    final_yaw_error = abs(true_states[-1, 3] - estimated_states[-1, 3])
    
    # Calculate average errors
    avg_pos_error = np.mean(np.sqrt((true_states[:, 0] - estimated_states[:, 0])**2 + 
                                   (true_states[:, 1] - estimated_states[:, 1])**2))
    avg_vel_error = np.mean(np.abs(true_states[:, 2] - estimated_states[:, 2]))
    
    # Calculate NIS consistency
    lidar_consistency = np.mean(np.array(ukf.nis_lidar) < 5.991)  # Should be close to 0.95 for proper tuning
    radar_consistency = np.mean(np.array(ukf.nis_radar) < 7.815)  # Should be close to 0.95 for proper tuning
    
    # Print results
    print("\nUKF Performance Statistics:")
    print("--------------------------")
    print(f"Final position error: {final_pos_error:.4f} m")
    print(f"Final velocity error: {final_vel_error:.4f} m/s")
    print(f"Final yaw angle error: {final_yaw_error:.4f} rad")
    print(f"Average position error: {avg_pos_error:.4f} m")
    print(f"Average velocity error: {avg_vel_error:.4f} m/s")
    print(f"LiDAR measurement consistency: {lidar_consistency:.4f} (ideal: 0.95)")
    print(f"Radar measurement consistency: {radar_consistency:.4f} (ideal: 0.95)")

# Main execution
if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Run UKF for LiDAR and Radar Sensor Fusion Simulation')
    parser.add_argument('--timesteps', type=int, default=200,
                        help='Number of simulation steps (default: 200)')
    parser.add_argument('--dt', type=float, default=0.1,
                        help='Time step size in seconds (default: 0.1)')
    parser.add_argument('--lidar_noise', type=float, nargs=2, default=[0.15, 0.15],
                        help='LiDAR measurement noise std [x, y] in meters (default: 0.15 0.15)')
    parser.add_argument('--radar_noise', type=float, nargs=3, default=[0.3, 0.03, 0.3],
                        help='Radar measurement noise std [range, angle, range_rate] (default: 0.3 0.03 0.3)')
    parser.add_argument('--process_noise', type=float, nargs=2, default=[2.0, 0.5],
                        help='Process noise std [acceleration, yaw_acceleration] (default: 2.0 0.5)')
    
    args = parser.parse_args()
    
    np.random.seed(42)  # For reproducibility
    
    print("Starting UKF LiDAR and Radar Sensor Fusion Simulation")
    print(f"Using {args.timesteps} time steps with dt={args.dt}")
    print(f"LiDAR noise: {args.lidar_noise}")
    print(f"Radar noise: {args.radar_noise}")
    print(f"Process noise: {args.process_noise}")
    
    # Run simulation
    true_states, lidar_measurements, radar_measurements, estimated_states, ukf = run_ukf_simulation(
        dt=args.dt, 
        n_steps=args.timesteps,
        std_lidar=args.lidar_noise,
        std_radar=args.radar_noise,
        process_noise=args.process_noise
    )
    
    # Plot results
    plot_results(true_states, lidar_measurements, radar_measurements, estimated_states, ukf)
    
    # Print statistics
    print_statistics(true_states, estimated_states, ukf)