package control.examples.pid.temperature_control;

import control.filter.LowPassFilter;
import control.pid.PID;

public class Main {
    public static void main(String[] args) {
        double ambient = 20.0;
        double feedForward = ambient * 0.02;

        // Temperature control with realistic thermal dynamics
        PID controller = new PID(0.5, 0.1, 0.02).withFeedForward(feedForward).withIntegralResetOnZeroCross()
                .withFilter(new LowPassFilter(0.8)).withOutputLimits(0.0, 1.0);

        double setpoint = 70.0;
        double temp = ambient;
        double dt = 0.05; // 50ms

        System.out.println("Temperature Control System");
        System.out.println("==========================");
        System.out.println(String.format("Ambient: %.1f°C, Target: %.1f°C\n", ambient, setpoint));
        System.out.printf("%-8s %-12s %-12s %-12s%n", "Time", "Temp(°C)", "Error(°C)", "Heater(%)");
        System.out.println("-----------------------------------------------");

        for (int i = 0; i <= 800; i++) {
            double time = i * dt;
            double heater = controller.calculate(setpoint, temp, dt);

            // Thermal model: heating power vs heat loss to ambient
            double heatInput = heater * 3000.0; // Watts (heater power)
            double heatLoss = 50.0 * (temp - ambient); // Heat dissipation (W/°C)
            temp += (heatInput - heatLoss) * 0.001 * dt; // Temperature change

            if (i % 40 == 0) {
                double error = setpoint - temp;
                System.out.printf("%.2f     %8.2f     %8.2f     %8.1f%%%n", time, temp, error, heater * 100);
            }
        }

        double finalError = Math.abs(setpoint - temp);
        System.out.println(String.format("\nFinal temperature: %.2f°C (target: %.1f°C)", temp, setpoint));
        System.out.println(String.format("Final error: %.2f°C (%.1f%%)", finalError, (finalError / setpoint) * 100));
        System.out.println("\nFeatures:");
        System.out.println("- Feedforward compensates for ambient temperature");
        System.out.println("- Integral reset prevents windup during large errors");
        System.out.println("- Filtered derivative handles noisy temperature sensor");
        System.out.println("- Output limited to 0-100% (heater can't cool)");
    }
}
