# PID Path-finding Simulation

A comprehensive 2D robotics simulator featuring differential drive robots, PID control systems, A* pathfinding, and real-time obstacle avoidance.

## 🤖 Features

### Core Robotics Concepts
- **Differential Drive Kinematics**: Realistic wheel-based robot movement
- **PID Control Systems**: Velocity-aware PID controllers for precise navigation
- **A* Pathfinding**: Intelligent route planning around obstacles
- **Sensor Simulation**: Distance sensors, proximity detection, and collision avoidance
- **Physics Engine**: 2D rigid body dynamics with friction and collision detection

### Advanced Control Systems
- **Precision Movement Controller**: Adaptive PID with velocity damping
- **Emergency Braking System**: Active braking forces for immediate stops
- **Obstacle Avoidance**: Multi-level reactive navigation system
- **Manual Override**: Seamless switching between autonomous and manual control

### Interactive Features
- **Click-to-Navigate**: Point-and-click autonomous navigation
- **Real-time Visualization**: Live display of robot state, sensors, and paths
- **Velocity Monitoring**: Real-time tracking of linear and angular velocities
- **Debug Information**: Comprehensive status display for learning and debugging

## 🎮 Controls

- **Arrow Keys**: Manual robot control (overrides autonomous mode)
- **Left Click**: Set autonomous navigation target (A* pathfinding)
- **Space Bar**: Emergency brake (active braking system)
- **ESC**: Exit simulation

## 🏗️ Architecture

```
src/
├── physics.py          # 2D physics engine and vector mathematics
├── robot.py            # Differential drive robot implementation
├── controllers.py      # PID controllers and navigation algorithms
├── pathfinding.py      # A* pathfinding and navigation grid
├── sensors.py          # Sensor simulation and data processing
├── environment.py      # Obstacle generation and boundary walls
└── visualization.py    # Real-time rendering and UI

examples/
└── basic_simulation.py # Main simulation demo
```

## 🚀 Quick Start

### Prerequisites
```bash
pip install pygame numpy
```

### Running the Simulation
```bash
cd examples
python basic_simulation.py
```

## 📚 Educational Value

This simulator demonstrates key robotics and control theory concepts:

### 1. **PID Control Theory**
- **Proportional**: Response proportional to current error
- **Integral**: Correction for accumulated past errors  
- **Derivative**: Prediction based on error rate of change
- **Velocity Damping**: Advanced technique to prevent overshooting

### 2. **Pathfinding Algorithms**
- **A* Search**: Optimal pathfinding with heuristics
- **Grid-based Navigation**: Environment discretization
- **Waypoint Following**: Breaking complex paths into manageable segments

### 3. **Robotics Fundamentals**
- **Forward Kinematics**: Converting wheel speeds to robot motion
- **Inverse Kinematics**: Converting desired motion to wheel speeds
- **Sensor Fusion**: Combining multiple sensor inputs for navigation
- **Reactive vs. Deliberative Control**: Balancing planning and reaction

### 4. **Real-time Systems**
- **60 FPS Simulation Loop**: Consistent timing for realistic physics
- **Control System Integration**: Multiple controllers working together
- **Emergency Procedures**: Safety systems and fail-safes

## 🎯 Technical Highlights

### Velocity-Aware PID Control
```python
# Traditional PID
output = kp * error + ki * integral + kd * derivative

# Enhanced with velocity feedback
output = kp * error + ki * integral + kd * derivative - kv * velocity_error
```

### A* Pathfinding Integration
- Automatic obstacle grid generation
- Real-time path replanning
- Smooth waypoint following with PID control

### Emergency Braking System
- Active force application opposing current motion
- Separate angular and linear braking coefficients
- Immediate response for safety-critical situations

## 🔧 Customization

### Tuning PID Parameters
```python
# In src/controllers.py - adjust for different behaviors
position_controller = VelocityAwarePIDController(
    kp=1.2,   # Aggressiveness of approach
    ki=0.05,  # Long-term error correction
    kd=0.3,   # Smoothing and prediction
    kv=0.7    # Velocity damping (prevents overshoot)
)
```

### Adjusting Robot Physics
```python
# In src/robot.py - modify friction and dynamics
friction_coefficient = 0.2           # Linear friction
angular_friction_coefficient = 1.5   # Angular friction (higher for stability)
```

## 🎓 Learning Outcomes

After exploring this simulator, you'll understand:
- How PID controllers work in practice
- Why velocity feedback is crucial for precision control
- How A* pathfinding integrates with real-time control
- The relationship between planning and reactive control
- Professional robotics software architecture patterns

## 🤝 Contributing

This project serves as an educational platform for robotics concepts. Contributions welcome for:
- Additional control algorithms
- New sensor types
- Advanced pathfinding techniques
- 3D visualization upgrades

## 📄 License

Open source educational project. Feel free to use for learning and teaching robotics concepts.

---

**Perfect for**: Robotics courses, control theory demonstrations, autonomous systems education, and as a foundation for more advanced robotics projects.