package control.examples.pid.temperature_control;

import control.filter.LowPassFilter;
import control.pid.PID;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        double ambient = 20.0;
        double feedForward = ambient * 0.02;

        PID controller = new PID(0.5, 0.1, 0.02, PID.withFeedForward(feedForward), PID.withIntegralResetOnZeroCross(),
                PID.withFilter(new LowPassFilter(0.2)));
        controller.setOutputLimits(0.0, 1.0);

        double setpoint = 70.0;
        double temp = ambient;

        for (int i = 0; i < 200; i++) {
            double heater = controller.calculate(setpoint, temp);
            double heatInput = heater * 2000.0;
            double heatLoss = 50.0 * (temp - ambient);
            temp += (heatInput - heatLoss) * 0.001;

            if (i % 20 == 0) {
                System.out.printf("step=%d temp=%.2f heater=%.2f%%%n", i, temp, heater * 100);
            }
            Thread.sleep(50);
        }
    }
}
