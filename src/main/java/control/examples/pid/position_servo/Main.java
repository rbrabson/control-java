package control.examples.pid.position_servo;

import control.pid.PID;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        PID controller = new PID(1.5, 0.2, 0.05, PID.withIntegralSumMax(10.0), PID.withStabilityThreshold(50.0));
        controller.setOutputLimits(-1.0, 1.0);

        double target = 90.0;
        double position = 0.0;
        double velocity = 0.0;

        for (int i = 0; i < 300; i++) {
            double command = controller.calculate(target, position);
            velocity += command * 3.0;
            velocity = Math.max(-180.0, Math.min(180.0, velocity));
            position += velocity * 0.01;

            if (i % 30 == 0) {
                System.out.printf("step=%d pos=%.2f vel=%.2f cmd=%.3f%n", i, position, velocity, command);
            }
            Thread.sleep(10);
        }
    }
}
