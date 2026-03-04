package control.examples.pid.basic_control_loop;

import control.pid.PID;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        PID controller = new PID(1.0, 0.1, 0.05, PID.withOutputLimits(-100, 100));

        double setpoint = 50.0;
        double position = 0.0;

        for (int i = 0; i < 100; i++) {
            double output = controller.calculate(setpoint, position);
            position += output * 0.01;

            if (i % 10 == 0) {
                double error = setpoint - position;
                System.out.printf("step=%d error=%.2f output=%.2f position=%.2f%n", i, error, output, position);
            }
            Thread.sleep(10);
        }
    }
}
