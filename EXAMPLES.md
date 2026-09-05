# Java Control Systems Examples

This document provides a comprehensive guide to the example programs included in the `control-java` package. These runnable examples demonstrate real-world control theory concepts, practical engineering patterns, and the configuration of controllers, filters, motion profiles, and lookup tables.

---

## Table of Contents

1. [Quick Start: Build & Run](#quick-start-build--run)
2. [Recommended Learning Path](#recommended-learning-path)
3. [Examples by Category](#examples-by-category)
   - [PID Controllers](#1-pid-controllers-comrbrabsoncontrolexamplespid)
   - [Feedforward Control](#2-feedforward-control-comrbrabsoncontrolexamplesfeedforward)
   - [Motion Profiling](#3-motion-profiling-comrbrabsoncontrolexamplesmotionprofile)
   - [Full-State Feedback](#4-full-state-feedback-comrbrabsoncontrolexamplesfeedback)
   - [Filters & State Estimation](#5-filters--state-estimation-comrbrabsoncontrolexamplesfilter)
   - [Interpolated Lookup Tables (InterpLUT)](#6-interpolated-lookup-tables-interplut-comrbrabsoncontrolexamplesinterplut)
4. [Cross-Cutting Patterns & Techniques](#cross-cutting-patterns--techniques)
5. [Quick Reference Matrix](#quick-reference-matrix)

---

## Quick Start: Build & Run

### 1. Build the Project

From the project root directory:

```bash
mvn compile
```

Or skip unit tests for a faster build:

```bash
mvn -DskipTests compile
```

### 2. Run Any Example

Execute any example class using standard Java CLI:

```bash
java -cp target/classes <fully.qualified.MainClass>
```

For example:

```bash
java -cp target/classes com.rbrabson.control.examples.pid.basic_control_loop.Main
```

---

## Recommended Learning Path

If you are new to control systems or this library, we recommend exploring the examples in the following progression:

1. **Foundations**: Start with `pid.basic_control_loop` to see standard proportional-integral-derivative behavior.
2. **Feedforward Fundamentals**: Explore `feedforward.basic`, `feedforward.arm`, and `feedforward.elevator` to understand physics-based open-loop control.
3. **Sensor Noise & Filtering**: Learn how `filter.lowpass` and `filter.basic` (Kalman) clean up measurements, and inspect `pid.filter_comparison` to see the effect on derivative calculations.
4. **Advanced PID Tuning & Robustness**: Review `pid.dampening`, `pid.motor_speed`, and `pid.temperature_control` to learn anti-windup strategies and stability thresholds.
5. **Smooth Trajectories**: Study `motionprofile.basic` and `motionprofile.triangle` for kinematic path generation.
6. **Integrated Control Strategies**: See `motionprofile.fullstate_control` and `interplut.adaptive_pid` to combine motion profiles, feedforward, full-state feedback, and dynamic gain scheduling.

---

## Examples by Category

### 1. PID Controllers (`com.rbrabson.control.examples.pid`)

PID (Proportional-Integral-Derivative) controllers calculate an output based on error between a reference setpoint and the measured state. The library provides advanced features including output clamping, integral anti-windup, stability thresholds, and derivative filtering.

#### `pid.basic_control_loop`
- **Class**: `com.rbrabson.control.examples.pid.basic_control_loop.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.pid.basic_control_loop.Main`
- **Application / Use Case**: General 1D position control (e.g., CNC axis, linear slide, steering angle).
- **Concepts Demonstrated**:
  - Proportional ($K_p$) response driving the system toward the target.
  - Integral ($K_i$) action eliminating steady-state offset over time.
  - Derivative ($K_d$) action providing damping against overshoot.
  - Simulating a continuous second-order plant using discrete time-steps (`dt`).

#### `pid.dampening`
- **Class**: `com.rbrabson.control.examples.pid.dampening.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.pid.dampening.Main`
- **Application / Use Case**: High-gain systems susceptible to overshoot or large setpoint steps (e.g., robotic arms moving between distant waypoints).
- **Concepts Demonstrated**:
  - Side-by-side comparison of **Basic PID**, **Filtered PID**, and **Filtered + Stability-Damped PID**.
  - Using `.withStabilityThreshold(threshold)` to suppress integral accumulation when raw derivative dynamics are large.
  - Preventing integral windup during large step transients while preserving steady-state precision.

#### `pid.filter_comparison`
- **Class**: `com.rbrabson.control.examples.pid.filter_comparison.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.pid.filter_comparison.Main`
- **Application / Use Case**: Systems with noisy feedback sensors (e.g., optical encoders, analog potentiometers, ultrasonic distance sensors).
- **Concepts Demonstrated**:
  - Compares raw unfiltered derivative calculations against `LowPassFilter` and `KalmanFilter` smoothing.
  - Explains how derivative noise amplification causes actuator jitter and motor heating in real hardware.
  - Shows that filtered controllers reach target accuracy while mitigating high-frequency chatter.

#### `pid.motor_speed`
- **Class**: `com.rbrabson.control.examples.pid.motor_speed.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.pid.motor_speed.Main`
- **Application / Use Case**: DC/BLDC motor velocity control, flywheel regulation, conveyor belt speed control (operating in hundreds or thousands of RPM).
- **Concepts Demonstrated**:
  - Gain scaling for large numerical state values (RPM).
  - Integral clamping via `.withIntegralSumMax(2000.0)` to limit maximum windup capacity.
  - Stability thresholding (`.withStabilityThreshold(200.0)`) to ignore aggressive transitions.
  - First-order motor response modeling ($ \tau \approx 0.25\text{s} $) with normalized output power mapping ($\pm 100\%$).

#### `pid.position_servo`
- **Class**: `com.rbrabson.control.examples.pid.position_servo.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.pid.position_servo.Main`
- **Application / Use Case**: Smart servos, gimbal axes, camera pan-tilt units where controller output directly dictates velocity.
- **Concepts Demonstrated**:
  - Proportional-only ($P$) control architecture where controller output commands velocity rather than force/acceleration.
  - Deterministic time-step simulation using `calculate(setpoint, state, dt)`.

#### `pid.temperature_control`
- **Class**: `com.rbrabson.control.examples.pid.temperature_control.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.pid.temperature_control.Main`
- **Application / Use Case**: 3D printer hotends, heated beds, chemical reactors, environmental chambers.
- **Concepts Demonstrated**:
  - Thermal physics simulation incorporating active heating power (Watts) and ambient heat dissipation ($W/^\circ\text{C}$).
  - Asymmetric output limits (`.withOutputLimits(0.0, 1.0)`) reflecting systems that can heat but cannot active-cool.
  - Static feedforward offset (`.withFeedForward(...)`) to support baseline thermal equilibrium.
  - Anti-windup via `.withIntegralResetOnZeroCross()` to clear accumulated integral error as soon as the target temperature is crossed.

---

### 2. Feedforward Control (`com.rbrabson.control.examples.feedforward`)

Feedforward calculates the predictive control effort required to achieve a desired kinematic state based on physical system models, rather than waiting for errors to occur.

Formula:
$$\text{Output} = k_S \cdot \text{sgn}(v \text{ or } a) + k_V \cdot v + k_A \cdot a + k_{Cos} \cdot \cos(\theta)$$

#### `feedforward.basic`
- **Class**: `com.rbrabson.control.examples.feedforward.basic.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.feedforward.basic.Main`
- **Application / Use Case**: Friction and inertia compensation for horizontal drivetrains, conveyor tracks, wheeled robots.
- **Concepts Demonstrated**:
  - Static friction gain ($k_S$): overcoming static stiction.
  - Velocity gain ($k_V$): overcoming viscous friction/back-EMF proportional to velocity.
  - Acceleration gain ($k_A$): overcoming inertial mass resistance ($F = ma$).

#### `feedforward.arm`
- **Class**: `com.rbrabson.control.examples.feedforward.arm.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.feedforward.arm.Main`
- **Application / Use Case**: Single-joint robot arms, pivoting linkages, wrist mechanisms, tilting mechanisms.
- **Concepts Demonstrated**:
  - Cosine gravity compensation via `.withCosineGain(kCos)`.
  - Torque dependency on angle: maximum holding torque required at horizontal ($0^\circ, \cos(0) = 1.0$) and zero gravity torque required at vertical ($90^\circ, \cos(90^\circ) = 0.0$).
  - Comparing feedforward commands with and without gravity compensation across a $180^\circ$ sweep.

#### `feedforward.elevator`
- **Class**: `com.rbrabson.control.examples.feedforward.elevator.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.feedforward.elevator.Main`
- **Application / Use Case**: Vertical elevators, linear lifters, scissor lifts, winch cables.
- **Concepts Demonstrated**:
  - Constant gravity compensation using $k_S$ (e.g., $9.81\text{ m/s}^2$) independent of vertical position.
  - Directional behavior: holding stationary at rest vs. moving/accelerating upward vs. downward.

#### `feedforward.crane`
- **Class**: `com.rbrabson.control.examples.feedforward.crane.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.feedforward.crane.Main`
- **Application / Use Case**: Heavy construction cranes, boom lifts, multi-stage excavators with variable boom angles.
- **Concepts Demonstrated**:
  - Combined compensation: static vertical lift baseline ($k_S = 15.7$) plus boom angle cosine torque ($k_{Cos} = 8.2$) alongside velocity and acceleration dynamics.

#### `feedforward.compare`
- **Class**: `com.rbrabson.control.examples.feedforward.compare.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.feedforward.compare.Main`
- **Application / Use Case**: Selecting the appropriate feedforward model for a physical mechanism.
- **Concepts Demonstrated**:
  - Direct side-by-side comparison of **Basic** (inertial only), **Elevator** (constant gravity), and **Arm** (angle-dependent gravity) controllers responding to identical kinematic inputs.

---

### 3. Motion Profiling (`com.rbrabson.control.examples.motionprofile`)

Motion profiles generate time-parameterized reference trajectories $(\text{position}, \text{velocity}, \text{acceleration})$ that obey kinematic velocity and acceleration constraints, ensuring smooth motion without mechanical shock.

#### `motionprofile.basic`
- **Class**: `com.rbrabson.control.examples.motionprofile.basic.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.motionprofile.basic.Main`
- **Application / Use Case**: Point-to-point path generation for 3D printers, CNC routers, automated pick-and-place gantry systems.
- **Concepts Demonstrated**:
  - Defining kinematic bounds using `Constraints(maxVelocity, maxAcceleration)`.
  - Trapezoidal motion phases: (1) Acceleration $\to$ (2) Constant-velocity cruise $\to$ (3) Deceleration $\to$ (4) At rest.
  - Querying profile state `calculate(t)` at arbitrary time offsets.

#### `motionprofile.triangle`
- **Class**: `com.rbrabson.control.examples.motionprofile.triangle.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.motionprofile.triangle.Main`
- **Application / Use Case**: Short-distance positioning, micro-stepping, quick indexing.
- **Concepts Demonstrated**:
  - Understands the mathematical boundary condition where a move is too short to reach `maxVelocity`, resulting in a triangular velocity profile.
  - Compares short-distance profile dynamics against long-distance trapezoidal profiles.

#### `motionprofile.fullstate_control`
- **Class**: `com.rbrabson.control.examples.motionprofile.fullstate_control.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.motionprofile.fullstate_control.Main`
- **Application / Use Case**: High-precision robotics, CNC machining, autonomous vehicle steering, flight control.
- **Concepts Demonstrated**:
  - State-of-the-art combined control architecture:
    1. **Trajectory Generator**: `MotionProfile` produces desired position, velocity, and acceleration at each time step.
    2. **Feedforward**: Injects planned acceleration directly into control output.
    3. **Full-State Feedback**: Simultaneously drives position and velocity errors to zero via gain vector $K = [k_{pos}, k_{vel}]$.
  - Demonstrates near-zero tracking error and minimal steady-state lag throughout the entire trajectory.

---

### 4. Full-State Feedback (`com.rbrabson.control.examples.feedback`)

Full-state feedback regulates multiple state variables simultaneously using a gain matrix/vector ($u = K \cdot (x_{ref} - x)$), offering superior damping and pole-placement control over single-variable PID.

#### `feedback.feedback_control`
- **Class**: `com.rbrabson.control.examples.feedback.feedback_control.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.feedback.feedback_control.Main`
- **Application / Use Case**: Inverted pendulum, balance bots, mass-spring-damper systems, drone altitude/attitude hold.
- **Concepts Demonstrated**:
  - Multi-variable state regulation with $K = [k_{pos}, k_{vel}]$.
  - How velocity feedback acts as active damping to eliminate position oscillations without requiring derivative approximations.
  - Closed-loop response simulation over time.

---

### 5. Filters & State Estimation (`com.rbrabson.control.examples.filter`)

Filters smooth sensor measurements, eliminate high-frequency noise, and estimate underlying system states.

#### `filter.lowpass`
- **Class**: `com.rbrabson.control.examples.filter.lowpass.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.filter.lowpass.Main`
- **Application / Use Case**: Sensor debouncing, smoothing current/voltage sensor data, accelerometer noise suppression.
- **Concepts Demonstrated**:
  - Exponential moving average smoothing: $y_k = \alpha \cdot x_k + (1 - \alpha) \cdot y_{k-1}$.
  - Comparative analysis of smoothing factors ($\alpha = 0.8, 0.5, 0.2$).
  - Explores the engineering trade-off: high $\alpha$ gives fast step response with less noise reduction; low $\alpha$ provides heavy smoothing at the cost of phase lag.

#### `filter.basic` (Kalman Filter with Regression)
- **Class**: `com.rbrabson.control.examples.filter.basic.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.filter.basic.Main`
- **Application / Use Case**: State estimation for moving objects, odometry tracking, radar/sonar range finding with trend estimation.
- **Concepts Demonstrated**:
  - 1D Kalman filter combining process covariance ($Q$), measurement noise covariance ($R$), and a sliding-window stack (`SizedStack`).
  - Uses `LinearRegression` over recent history to predict trends for moving states rather than assuming a static process.
  - Dynamic adaptation of the Kalman gain ($K$) to balance model prediction versus noisy measurements.

---

### 6. Interpolated Lookup Tables (InterpLUT) (`com.rbrabson.control.examples.interplut`)

`InterpLUT` provides piecewise monotone linear interpolation between defined calibration points, enabling non-linear mappings and dynamic parameter scheduling.

#### `interplut.basic`
- **Class**: `com.rbrabson.control.examples.interplut.basic.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.interplut.basic.Main`
- **Application / Use Case**: Engine throttle mapping, non-linear actuator calibration, joystick deadband/sensitivity curves.
- **Concepts Demonstrated**:
  - Building lookup tables using fluent `.withPoint(x, y)` builders.
  - Piecewise linear interpolation between discrete calibration points.
  - Strict input domain validation and out-of-bounds `IllegalArgumentException` safety checks.

#### `interplut.temperature`
- **Class**: `com.rbrabson.control.examples.interplut.temperature.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.interplut.temperature.Main`
- **Application / Use Case**: Temperature-compensated pressure sensors, strain gauges, battery capacity estimation across ambient temperatures.
- **Concepts Demonstrated**:
  - Defining non-linear temperature scale factors across operating temperature ranges ($0^\circ\text{C}$ to $60^\circ\text{C}$).
  - Applying correction factors to raw sensor measurements to maintain calibration across thermal shifts.

#### `interplut.adaptive_pid`
- **Class**: `com.rbrabson.control.examples.interplut.adaptive_pid.Main`
- **Run**: `java -cp target/classes com.rbrabson.control.examples.interplut.adaptive_pid.Main`
- **Application / Use Case**: Nonlinear systems, magnetic levitation, long-stroke actuators requiring both high slew rates and fine settling without hunting.
- **Concepts Demonstrated**:
  - **Gain Scheduling / Adaptive Control**: Dynamically adjusting controller gain ($K_p$) based on instantaneous error magnitude via an `InterpLUT`.
  - High proportional gain for large errors (fast rise time) and lower proportional gain for small errors (smooth settling without overshoot).

---

## Cross-Cutting Patterns & Techniques

When developing control applications with `control-java`, consider these recurring patterns illustrated across the examples:

### 1. Feedforward + Feedback Synergy
- Feedforward calculates baseline effort directly from kinematic intentions ($v$, $a$, $\theta$), doing the "heavy lifting" without delay.
- Feedback (PID or Full-State Feedback) compensates only for unmodeled disturbances and measurement errors.
- *Reference*: `motionprofile.fullstate_control` and `pid.temperature_control`.

### 2. Preventing Integral Windup
- Use **Output Limits** (`.withOutputLimits(min, max)`) to prevent saturated actuators.
- Use **Integral Sum Limits** (`.withIntegralSumMax(max)`) for high-range states like RPM.
- Use **Stability Thresholds** (`.withStabilityThreshold(threshold)`) to pause integration during rapid transitions.
- Use **Zero-Crossing Resets** (`.withIntegralResetOnZeroCross()`) to clear integration when the error changes sign.
- *Reference*: `pid.motor_speed`, `pid.dampening`, `pid.temperature_control`.

### 3. Mitigating Sensor Noise on Derivative Terms
- Direct numerical differentiation ($\Delta e / \Delta t$) amplifies high-frequency noise.
- Attach a `LowPassFilter` or `KalmanFilter` to the PID controller using `.withFilter(filter)`.
- *Reference*: `pid.filter_comparison`, `filter.lowpass`, `filter.basic`.

---

## Quick Reference Matrix

| Example Entrypoint | Category | Key Components Used | Target Physical System / Problem |
|---|---|---|---|
| `pid.basic_control_loop.Main` | PID | `PID` | 1D Position regulation |
| `pid.dampening.Main` | PID | `PID`, `LowPassFilter` | Transient step response & stability thresholds |
| `pid.filter_comparison.Main` | PID | `PID`, `LowPassFilter`, `KalmanFilter` | Derivative noise rejection comparison |
| `pid.motor_speed.Main` | PID | `PID`, `LowPassFilter` | DC/BLDC motor velocity (RPM) control |
| `pid.position_servo.Main` | PID | `PID` | Angular servo position (P-only velocity command) |
| `pid.temperature_control.Main` | PID | `PID`, `LowPassFilter` | Thermal heating process with ambient dissipation |
| `feedforward.basic.Main` | FeedForward | `FeedForward` | Linear inertia & friction compensation ($k_S, k_V, k_A$) |
| `feedforward.arm.Main` | FeedForward | `FeedForward` | Rotating arm with cosine gravity torque ($k_{Cos}$) |
| `feedforward.elevator.Main` | FeedForward | `FeedForward` | Vertical lift with constant gravity offset ($k_S$) |
| `feedforward.crane.Main` | FeedForward | `FeedForward` | Crane with vertical load + boom angle cosine torque |
| `feedforward.compare.Main` | FeedForward | `FeedForward` | Comparative analysis of feedforward models |
| `motionprofile.basic.Main` | Motion Profile | `MotionProfile`, `Constraints`, `State` | Trapezoidal trajectory profile generation |
| `motionprofile.triangle.Main` | Motion Profile | `MotionProfile`, `Constraints`, `State` | Short-move triangular velocity profiles |
| `motionprofile.fullstate_control.Main` | Motion Profile | `MotionProfile`, `FullStateFeedback` | Trajectory tracking with full-state feedback + FF |
| `feedback.feedback_control.Main` | Feedback | `FullStateFeedback` | Multi-variable state regulation ($K = [k_{pos}, k_{vel}]$) |
| `filter.lowpass.Main` | Filter | `LowPassFilter` | Exponential moving average smoothing ($\alpha$) |
| `filter.basic.Main` | Filter | `KalmanFilter`, `LinearRegression` | 1D Kalman filter with regression trend estimation |
| `interplut.basic.Main` | InterpLUT | `InterpLUT` | Piecewise monotonic linear lookup table & calibration |
| `interplut.temperature.Main` | InterpLUT | `InterpLUT` | Non-linear sensor temperature compensation table |
| `interplut.adaptive_pid.Main` | InterpLUT | `InterpLUT`, `PID` | Dynamic gain scheduling based on error magnitude |
