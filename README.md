# control-java

Java implementation of the control systems library from the Go `control` project.

This library provides reusable control primitives for robotics and automation workloads:

- PID control with advanced anti-windup and filtering options
- Feedforward control with gravity and cosine compensation
- Full-state feedback for multi-variable control
- Motion profile generation (trapezoidal/triangular)
- Filtering utilities (low-pass and Kalman)
- Monotone cubic interpolation lookup tables (InterpLUT)

## Choosing the Right Control Mechanism

Use this rule of thumb:

- **PID** when you need to hold or track one measured variable (speed, angle, position) and reject disturbances.
- **Feedforward** when you can model what effort is needed ahead of time (velocity/acceleration/gravity effects).
- **Full-state feedback** when one actuator is influenced by multiple coupled states and you want one control law over the full state vector.
- **Motion profile** when setpoint changes should be smooth and constrained by max velocity/acceleration.
- **Filters (Low-pass/Kalman)** when measurements are noisy or intermittent and raw values cause unstable control.
- **InterpLUT** when a relationship is nonlinear and best represented by measured calibration points.

In most real systems, you combine several of these:

1. **Reference generation**: create feasible position/velocity/acceleration goals (motion profile or operator command shaping).
2. **Model term**: compute expected effort (feedforward and/or LUT lookup).
3. **Error correction**: add PID or full-state feedback correction from sensors.
4. **Sensor conditioning**: filter noisy measurements before they drive control terms.

### Example: Shooter + Spindexer + Transfer + Flywheel + Hood with Camera Range

If a camera provides target pose/distance (x/y/z), a practical architecture is:

1. **Range and angle estimation**
   - Compute scalar distance from x/y/z (or use z directly if already range-aligned).
   - Filter this value (`LowPassFilter` or `KalmanFilter`) to reduce shot-to-shot jitter.
2. **Map distance to mechanism goals**
   - Use `InterpLUT` for `distance -> flywheel RPM`.
   - Use another `InterpLUT` for `distance -> hood angle`.
   - Optional third LUT for `distance -> expected flight time` to support lead compensation.
3. **Flywheel control**
   - Run velocity **PID** on measured wheel speed.
   - Add **FeedForward** for target wheel velocity/acceleration to improve spin-up response.
4. **Hood control**
   - Use position **PID** for hood angle.
   - Add cosine **FeedForward** (`kCos`) if gravity load changes with angle.
   - Optionally profile hood moves with `MotionProfile` to avoid overshoot and linkage shock.
5. **Spindexer/transfer sequencing**
   - Control each conveyor speed with simple PID (or open-loop if characterized well).
   - Gate feeding with logic: only feed when flywheel speed error and hood angle error are both within tolerance for a minimum dwell time.
6. **Whole-system behavior**
   - Recompute goals continuously from filtered camera distance.
   - Keep actuation robust when vision drops out by holding last valid target briefly and timing out to a safe fallback mode.

This is a strong pattern: **LUT for aiming goals, feedforward for predicted effort, PID for final correction, filters for noisy vision**.

### Other Real-World Patterns

- **Elevator or linear slide**: motion profile position target + position PID + velocity/acceleration feedforward.
- **Single-joint arm**: profile angle target + PID + cosine feedforward for gravity compensation.
- **Turret or yaw axis with noisy vision**: filtered heading error + PID, optionally profile large slews before fine-lock.
- **Drive velocity control**: wheel-speed PID + feedforward (`kV`, `kA`) for fast acceleration tracking.
- **Temperature process (heater/chamber)**: PID with filtered sensor input; feedforward when load changes are predictable.
- **Coupled states (e.g., balancing + velocity)**: full-state feedback over angle/rate/velocity states, potentially with separate outer-loop setpoint generation.

## Project Status

This repository currently targets:

- Java 17
- Maven build
- JUnit 5 tests

The core library and example programs are in place and compile successfully.

## Requirements

- JDK 17+
- Maven 3.8+

## Build and Test

From the repository root:

```bash
mvn compile
mvn test
```

Compile quickly without tests:

```bash
mvn -DskipTests compile
```

## Project Layout

- `src/main/java/com/rbrabson/control/feedback` — full-state feedback controller
- `src/main/java/com/rbrabson/control/feedforward` — feedforward controller
- `src/main/java/com/rbrabson/control/filter` — filter interface and implementations
- `src/main/java/com/rbrabson/control/interplut` — interpolating lookup table
- `src/main/java/com/rbrabson/control/motionprofile` — motion profile generation
- `src/main/java/com/rbrabson/control/pid` — PID controller
- `src/main/java/com/rbrabson/control/examples` — runnable example programs
- `src/test/java/com/rbrabson/control` — package tests

Example run commands are listed in [EXAMPLES.md](EXAMPLES.md).

## Packages

### PID (`com.rbrabson.control.pid`)

Main class: `PID`

Supports:

- Proportional/integral/derivative control
- Output clamping (`.withOutputLimits(...)`)
- Feedforward term (`.withFeedForward(...)`)
- Integral reset on zero crossing (`.withIntegralResetOnZeroCross()`)
- Integral cap (`.withIntegralSumMax(...)`)
- Stability threshold to suppress integral accumulation during high dynamics (`.withStabilityThreshold(...)`)
- Derivative filtering via pluggable `Filter` (`.withFilter(...)`)
- Optional dampening-derived `kd` (`.withDampening(...)`)

Configuration uses fluent methods that return copies to support method chaining.

Quick start:

```java
import com.rbrabson.control.pid.PID;

PID controller = new PID(1.0, 0.1, 0.05)
    .withOutputLimits(-100.0, 100.0);

double output = controller.calculate(50.0, 42.5);
```

### Feedforward (`com.rbrabson.control.feedforward`)

Main class: `FeedForward`

Model:

- `kV * velocity + kA * acceleration + kCos * cos(position)`

Configurable via fluent method:

- `.withCosineGain(...)` (for gravity/cosine compensation)

Quick start:

```java
import com.rbrabson.control.feedforward.FeedForward;

FeedForward ff = new FeedForward(0.0, 1.2, 0.3)
    .withCosineGain(2.5);

double u = ff.calculate(Math.PI / 4.0, 1.5, 0.2);
```

### Feedback (`com.rbrabson.control.feedback`)

Main class: `FullStateFeedback`

Implements dot-product full-state control:

- Computes `error = setpoint - measurement` element-wise
- Returns `dot(error, gain)`

Quick start:

```java
import com.rbrabson.control.feedback.FullStateFeedback;

FullStateFeedback fsf = new FullStateFeedback(new double[]{1.5, 0.3});
double out = fsf.calculate(
    new double[]{10.0, 0.0},
    new double[]{8.0, 1.0});
```

### Filters (`com.rbrabson.control.filter`)

Interface: `Filter`

- `double estimate(double measurement)`
- `void reset()`
- `double getGain()`

Implementations:

- `LowPassFilter` — first-order smoothing filter
- `KalmanFilter` — scalar Kalman filter with internal linear regression prediction

Low-pass example:

```java
import com.rbrabson.control.filter.LowPassFilter;

LowPassFilter lpf = new LowPassFilter(0.6);
double filtered = lpf.estimate(12.0);
```

### InterpLUT (`com.rbrabson.control.interplut`)

Main class: `InterpLUT`

Features:

- Add control points using fluent methods (`.withPoint(x, y)`)
- Build the spline via `.build()` or automatically on first `get()` call
- Evaluate with `get(input)`
- Validates duplicate X values and out-of-range requests

Quick start:

```java
import com.rbrabson.control.interplut.InterpLUT;

InterpLUT lut = new InterpLUT()
    .withPoint(0, 0.0)
    .withPoint(100, 1.0)
    .withPoint(200, 1.8)
    .build();

double y = lut.get(150);
```

### Motion Profile (`com.rbrabson.control.motionprofile`)

Main classes:

- `Constraints` (`maxVelocity`, `maxAcceleration`)
- `State` (`position`, `velocity`, `acceleration`, `time`)
- `MotionProfile`

`MotionProfile` computes trapezoidal or triangular profiles depending on motion distance and constraints.

Useful methods:

- `calculate(t)` — state at time `t`
- `totalTime()` — total profile duration
- `isFinished(t)` — completion check
- `timeLeftUntil(position)` — time to target position

Quick start:

```java
import com.rbrabson.control.motionprofile.Constraints;
import com.rbrabson.control.motionprofile.MotionProfile;
import com.rbrabson.control.motionprofile.State;

MotionProfile profile = new MotionProfile(
    new Constraints(2.0, 1.0),
    new State(0.0, 0.0, 0.0, 0.0),
    new State(5.0, 0.0, 0.0, 0.0)
);

State s = profile.calculate(0.5);
```

## Examples

All runnable examples are under `src/main/java/com/rbrabson/control/examples`.

You can run any example after compile:

```bash
mvn -DskipTests compile
java -cp target/classes com.rbrabson.control.examples.pid.basic_control_loop.Main
```

Complete command list: [EXAMPLES.md](EXAMPLES.md).

## Error Handling Notes

- Most invalid inputs are reported as `IllegalArgumentException` or `IllegalStateException`.
- `FullStateFeedback` requires matching vector dimensions.
- `LowPassFilter` gain must be in `(0, 1)`.
- `KalmanFilter` requires non-negative covariance and positive history size.
- `InterpLUT.get(...)` requires in-range input.

## Testing

Tests live under `src/test/java/com/rbrabson/control` and currently cover core behavior in all package areas.

Run:

```bash
mvn test
```

## License

See [LICENSE](../control/LICENSE) in the source Go project and your local project licensing policy for Java distribution decisions.
