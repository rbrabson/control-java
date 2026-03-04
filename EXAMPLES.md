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

InterpLUT examples use the options pattern (for example: `new InterpLUT(add(...), add(...))`).

- `java -cp target/classes control.examples.feedback.feedback_control.Main`
- `java -cp target/classes control.examples.feedforward.arm.Main`
- `java -cp target/classes control.examples.feedforward.basic.Main`
- `java -cp target/classes control.examples.feedforward.compare.Main`
- `java -cp target/classes control.examples.feedforward.crane.Main`
- `java -cp target/classes control.examples.feedforward.elevator.Main`
- `java -cp target/classes control.examples.filter.basic.Main`
- `java -cp target/classes control.examples.filter.lowpass.Main`
- `java -cp target/classes control.examples.interplut.adaptive_pid.Main`
- `java -cp target/classes control.examples.interplut.basic.Main`
- `java -cp target/classes control.examples.interplut.temperature.Main`
- `java -cp target/classes control.examples.motionprofile.basic.Main`
- `java -cp target/classes control.examples.motionprofile.fullstate_control.Main`
- `java -cp target/classes control.examples.motionprofile.triangle.Main`
- `java -cp target/classes control.examples.pid.basic_control_loop.Main`
- `java -cp target/classes control.examples.pid.dampening.Main`
- `java -cp target/classes control.examples.pid.filter_comparison.Main`
- `java -cp target/classes control.examples.pid.motor_speed.Main`
- `java -cp target/classes control.examples.pid.position_servo.Main`
- `java -cp target/classes control.examples.pid.temperature_control.Main`
