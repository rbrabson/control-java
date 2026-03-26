# Java Examples

This project includes Java example programs that mirror the Go examples.

## Build Once

From the project root:

```bash
mvn -DskipTests compile
```

## Run Any Example

Use:

```bash
java -cp target/classes <fully.qualified.MainClass>
```

## Example Entrypoints

- `java -cp target/classes com.rbrabson.control.examples.feedback.feedback_control.Main`
- `java -cp target/classes com.rbrabson.control.examples.feedforward.arm.Main`
- `java -cp target/classes com.rbrabson.control.examples.feedforward.basic.Main`
- `java -cp target/classes com.rbrabson.control.examples.feedforward.compare.Main`
- `java -cp target/classes com.rbrabson.control.examples.feedforward.crane.Main`
- `java -cp target/classes com.rbrabson.control.examples.feedforward.elevator.Main`
- `java -cp target/classes com.rbrabson.control.examples.filter.basic.Main`
- `java -cp target/classes com.rbrabson.control.examples.filter.lowpass.Main`
- `java -cp target/classes com.rbrabson.control.examples.interplut.adaptive_pid.Main`
- `java -cp target/classes com.rbrabson.control.examples.interplut.basic.Main`
- `java -cp target/classes com.rbrabson.control.examples.interplut.temperature.Main`
- `java -cp target/classes com.rbrabson.control.examples.motionprofile.basic.Main`
- `java -cp target/classes com.rbrabson.control.examples.motionprofile.fullstate_control.Main`
- `java -cp target/classes com.rbrabson.control.examples.motionprofile.triangle.Main`
- `java -cp target/classes com.rbrabson.control.examples.pid.basic_control_loop.Main`
- `java -cp target/classes com.rbrabson.control.examples.pid.dampening.Main`
- `java -cp target/classes com.rbrabson.control.examples.pid.filter_comparison.Main`
- `java -cp target/classes com.rbrabson.control.examples.pid.motor_speed.Main`
- `java -cp target/classes com.rbrabson.control.examples.pid.position_servo.Main`
- `java -cp target/classes com.rbrabson.control.examples.pid.temperature_control.Main`

### FeedForward Examples

- **Arm feedforward with cosine compensation for gravity** (cosine gain = 2.5 N·m)
  - The cosine gain compensates for gravity's varying effect at different angles
  - Example usage:
    - FeedForward armWithGravity = new FeedForward(0.0, 1.0, 0.2).withCosineGain(2.5);

- **Crane with both constant and cosine compensation**
  - The first argument is the constant (static) gain, the second is velocity gain, the third is acceleration gain.
  - Cosine compensation for boom angle (horizontal position affects torque) is set with .withCosineGain(...)
  - Example usage:
    - FeedForward craneFF = new FeedForward(15.7, 1.1, 0.25).withCosineGain(8.2);

- **Elevator feedforward with constant gravity compensation** (static gain = 9.81 m/s²)
  - Example usage:
    - FeedForward elevatorWithGravity = new FeedForward(9.81, 0.9, 0.2);
