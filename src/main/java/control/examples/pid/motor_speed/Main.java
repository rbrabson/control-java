package control.examples.pid.motor_speed;

import control.filter.LowPassFilter;
import control.pid.PID;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        PID controller = new PID(0.8, 0.1, 0.02, PID.withIntegralSumMax(10.0), PID.withStabilityThreshold(50.0),
                PID.withFilter(new LowPassFilter(0.1)));
        controller.setOutputLimits(-1.0, 1.0);

        double setpointRpm = 2000.0;
        double speed = 0.0;

        for (int i = 0; i < 200; i++) {
            double power = controller.calculate(setpointRpm, speed);
            speed += (power * 3000.0 - speed) * 0.02;
            if (i % 20 == 0) {
                System.out.printf("step=%d speed=%.1f power=%.3f%n", i, speed, power);
            }
            Thread.sleep(10);
        }
    }
}
