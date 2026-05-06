"""
AGV UWB-TDOA EKF Tracking Simulation

This script simulates a vehicle moving on a rectangular inner track and estimates
its position using UWB TDOA measurements and an Extended Kalman Filter.

Coordinate convention:
- x axis: right direction
- y axis: upward direction
- vehicle starts from the lower-left corner of the inner rectangle
- clockwise motion in Cartesian coordinates: up -> right -> down -> left
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class SimulationConfig:
    """Main simulation parameters."""

    dt: float = 0.2
    speed: float = 20.0
    total_time: float = 80.0
    process_noise_std: float = 0.8
    tdoa_noise_std_m: float = 0.8
    random_seed: int = 42


@dataclass
class GeometryConfig:
    """Sensor field and track geometry."""

    outer_width: float = 500.0
    outer_height: float = 300.0
    inner_left: float = 50.0
    inner_bottom: float = 50.0
    inner_width: float = 400.0
    inner_height: float = 200.0

    @property
    def sensors(self) -> np.ndarray:
        """Four UWB sensors on the corners of the outer rectangle."""
        return np.array(
            [
                [0.0, 0.0],
                [self.outer_width, 0.0],
                [self.outer_width, self.outer_height],
                [0.0, self.outer_height],
            ],
            dtype=float,
        )

    @property
    def track_corners(self) -> np.ndarray:
        """Inner rectangular track corners.

        Order is chosen for clockwise motion in the Cartesian coordinate system:
        lower-left -> upper-left -> upper-right -> lower-right -> lower-left.
        """
        x0 = self.inner_left
        y0 = self.inner_bottom
        x1 = self.inner_left + self.inner_width
        y1 = self.inner_bottom + self.inner_height
        return np.array(
            [
                [x0, y0],
                [x0, y1],
                [x1, y1],
                [x1, y0],
            ],
            dtype=float,
        )


def generate_rectangular_motion(
    geometry: GeometryConfig,
    config: SimulationConfig,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate true states for a vehicle moving around the inner rectangle.

    Returns:
        times: shape (N,)
        true_states: shape (N, 4), columns [x, y, vx, vy]
    """
    corners = geometry.track_corners
    dt = config.dt
    speed = config.speed
    times = np.arange(0.0, config.total_time + dt, dt)

    # Segment list: start point, direction vector, segment length.
    segments = []
    for i in range(len(corners)):
        start = corners[i]
        end = corners[(i + 1) % len(corners)]
        vector = end - start
        length = float(np.linalg.norm(vector))
        direction = vector / length
        duration = length / speed
        segments.append((start, direction, length, duration))

    lap_duration = sum(segment[3] for segment in segments)
    states = []

    for t in times:
        tau = t % lap_duration
        elapsed = 0.0

        for start, direction, length, duration in segments:
            if tau <= elapsed + duration:
                local_t = tau - elapsed
                position = start + direction * speed * local_t
                velocity = direction * speed
                states.append([position[0], position[1], velocity[0], velocity[1]])
                break
            elapsed += duration
        else:
            # Numerical fallback at lap boundary.
            start, direction, _, _ = segments[-1]
            states.append([start[0], start[1], direction[0] * speed, direction[1] * speed])

    return times, np.array(states, dtype=float)


def tdoa_measurement(position: np.ndarray, sensors: np.ndarray) -> np.ndarray:
    """Return range-difference TDOA measurement relative to sensor 0.

    z_i = distance(sensor_i) - distance(sensor_0), i = 1, 2, 3
    """
    distances = np.linalg.norm(sensors - position, axis=1)
    return distances[1:] - distances[0]


def tdoa_jacobian(position: np.ndarray, sensors: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Jacobian matrix of the range-difference measurement model.

    The full EKF state is [x, y, vx, vy]. TDOA depends only on position,
    so velocity derivatives are zero.
    """
    reference_sensor = sensors[0]
    diff_ref = position - reference_sensor
    dist_ref = max(float(np.linalg.norm(diff_ref)), eps)
    grad_ref = diff_ref / dist_ref

    jacobian = np.zeros((len(sensors) - 1, 4), dtype=float)

    for row, sensor in enumerate(sensors[1:]):
        diff_i = position - sensor
        dist_i = max(float(np.linalg.norm(diff_i)), eps)
        grad_i = diff_i / dist_i
        jacobian[row, 0:2] = grad_i - grad_ref

    return jacobian


def run_ekf(
    measurements: np.ndarray,
    sensors: np.ndarray,
    initial_state: np.ndarray,
    config: SimulationConfig,
) -> np.ndarray:
    """Run EKF estimation for TDOA measurements."""
    dt = config.dt

    f_matrix = np.array(
        [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )

    q = config.process_noise_std**2
    q_matrix = q * np.array(
        [
            [dt**4 / 4.0, 0.0, dt**3 / 2.0, 0.0],
            [0.0, dt**4 / 4.0, 0.0, dt**3 / 2.0],
            [dt**3 / 2.0, 0.0, dt**2, 0.0],
            [0.0, dt**3 / 2.0, 0.0, dt**2],
        ],
        dtype=float,
    )

    r_matrix = (config.tdoa_noise_std_m**2) * np.eye(len(sensors) - 1)

    state = initial_state.astype(float).copy()
    covariance = np.diag([25.0, 25.0, 16.0, 16.0])
    estimates = []

    for z in measurements:
        # Prediction step
        predicted_state = f_matrix @ state
        predicted_covariance = f_matrix @ covariance @ f_matrix.T + q_matrix

        # Update step
        predicted_position = predicted_state[:2]
        predicted_measurement = tdoa_measurement(predicted_position, sensors)
        h_matrix = tdoa_jacobian(predicted_position, sensors)

        innovation = z - predicted_measurement
        innovation_covariance = h_matrix @ predicted_covariance @ h_matrix.T + r_matrix
        kalman_gain = predicted_covariance @ h_matrix.T @ np.linalg.inv(innovation_covariance)

        state = predicted_state + kalman_gain @ innovation
        covariance = (np.eye(4) - kalman_gain @ h_matrix) @ predicted_covariance

        estimates.append(state.copy())

    return np.array(estimates, dtype=float)


def make_noisy_measurements(
    true_states: np.ndarray,
    sensors: np.ndarray,
    config: SimulationConfig,
) -> np.ndarray:
    """Generate noisy TDOA range-difference measurements."""
    rng = np.random.default_rng(config.random_seed)
    clean_measurements = np.array(
        [tdoa_measurement(state[:2], sensors) for state in true_states],
        dtype=float,
    )
    noise = rng.normal(
        loc=0.0,
        scale=config.tdoa_noise_std_m,
        size=clean_measurements.shape,
    )
    return clean_measurements + noise


def calculate_rmse(true_positions: np.ndarray, estimated_positions: np.ndarray) -> float:
    """Calculate 2D position RMSE."""
    errors = true_positions - estimated_positions
    squared_distances = np.sum(errors**2, axis=1)
    return float(np.sqrt(np.mean(squared_distances)))


def plot_results(
    true_states: np.ndarray,
    estimated_states: np.ndarray,
    geometry: GeometryConfig,
    output_path: Path,
) -> None:
    """Create and save a result plot."""
    sensors = geometry.sensors
    track = geometry.track_corners
    closed_track = np.vstack([track, track[0]])

    output_path.parent.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(10, 6))
    plt.plot(true_states[:, 0], true_states[:, 1], label="True path", linewidth=2)
    plt.plot(estimated_states[:, 0], estimated_states[:, 1], label="EKF estimate", linestyle="--")
    plt.scatter(sensors[:, 0], sensors[:, 1], marker="^", s=120, label="UWB sensors")
    plt.plot(closed_track[:, 0], closed_track[:, 1], linestyle=":", label="Inner track")

    outer_x = [0, geometry.outer_width, geometry.outer_width, 0, 0]
    outer_y = [0, 0, geometry.outer_height, geometry.outer_height, 0]
    plt.plot(outer_x, outer_y, linestyle="-.", label="Outer field")

    for index, sensor in enumerate(sensors):
        plt.text(sensor[0] + 5, sensor[1] + 5, f"S{index}")

    plt.title("AGV Position Tracking with UWB TDOA and EKF")
    plt.xlabel("x position [m]")
    plt.ylabel("y position [m]")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=200)
    plt.close()


def main() -> None:
    """Run the complete simulation."""
    config = SimulationConfig()
    geometry = GeometryConfig()
    sensors = geometry.sensors

    times, true_states = generate_rectangular_motion(geometry, config)
    measurements = make_noisy_measurements(true_states, sensors, config)

    # Start the EKF close to the known starting point, with a small intentional error.
    initial_state = true_states[0] + np.array([3.0, -2.0, 1.0, -1.0])
    estimated_states = run_ekf(measurements, sensors, initial_state, config)

    rmse = calculate_rmse(true_states[:, :2], estimated_states[:, :2])
    mean_abs_error = np.mean(np.linalg.norm(true_states[:, :2] - estimated_states[:, :2], axis=1))

    output_path = Path(__file__).resolve().parents[1] / "outputs" / "agv_uwb_tdoa_ekf_result.png"
    plot_results(true_states, estimated_states, geometry, output_path)

    print("AGV UWB-TDOA EKF simulation completed.")
    print(f"Number of time steps: {len(times)}")
    print(f"Position RMSE: {rmse:.3f} m")
    print(f"Mean absolute position error: {mean_abs_error:.3f} m")
    print(f"Result plot saved to: {output_path}")


if __name__ == "__main__":
    main()
