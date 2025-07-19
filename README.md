# Velocity Command Generation for AMR Navigation

This repository contains the implementation of a velocity command generation system for Autonomous Mobile Robots (AMRs). The system is designed to calculate appropriate velocity commands based on the robot's current state and navigation goals.

## Key Features

- Pure Python implementation without ROS dependencies
- Velocity command generation based on:
  - Current robot pose (position and orientation)
  - Target waypoint
  - Dynamic constraints (max velocity, acceleration)
- Simple and modular architecture for easy integration
- Configurable parameters for different robot kinematics

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/AMR-Vanguard/Velocity_command_Generation.git
   cd Velocity_command_Generation
   ```

2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

The main velocity command generator can be used as follows:

```python
from velocity_generator import VelocityCommandGenerator

# Initialize with robot parameters
generator = VelocityCommandGenerator(
    max_linear_velocity=0.5,  # m/s
    max_angular_velocity=1.0,  # rad/s
    linear_acceleration=0.1,  # m/s²
    angular_acceleration=0.2   # rad/s²
)

# Generate velocity commands
current_pose = [x, y, theta]  # Current robot pose [m, m, rad]
target_waypoint = [x_goal, y_goal]  # Target position [m, m]

linear_vel, angular_vel = generator.generate_commands(current_pose, target_waypoint)
```

## Configuration

The system can be configured by modifying the parameters in `config/parameters.yaml`:

```yaml
robot_parameters:
  max_linear_velocity: 0.5    # Maximum linear velocity (m/s)
  max_angular_velocity: 1.0   # Maximum angular velocity (rad/s)
  linear_acceleration: 0.1    # Linear acceleration (m/s²)
  angular_acceleration: 0.2   # Angular acceleration (rad/s²)
  lookahead_distance: 0.3     # Lookahead distance for pure pursuit (m)
```

## Examples

See the `examples/` directory for complete usage examples, including:
- Basic waypoint following
- Path tracking
- Dynamic obstacle avoidance integration

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- AMR Vanguard team members
- Contributors to open-source robotics algorithms
