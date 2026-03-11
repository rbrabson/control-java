# control-java

Java implementation of the control systems library from the Go `control` project.

This library provides reusable control primitives for robotics and automation workloads:

- PID control with advanced anti-windup and filtering options
- Feedforward control with gravity and cosine compensation
- Full-state feedback for multi-variable control
- Motion profile generation (trapezoidal/triangular)
- Filtering utilities (low-pass and Kalman)
- Monotone cubic interpolation lookup tables (InterpLUT)

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

- `kV * velocity + kA * acceleration + kG + kCos * cos(position)`

Configurable via fluent methods:

- `.withGravityGain(...)`
- `.withCosineGain(...)`

Quick start:

```java
import com.rbrabson.control.feedforward.FeedForward;

FeedForward ff = new FeedForward(0.0, 1.2, 0.3)
    .withGravityGain(9.81)
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
