package control.examples.pid.motor_speed;

import control.filter.LowPassFilter;
import control.pid.PID;

public class Main {
    public static void main(String[] args) {
        // Motor speed control with filtering and advanced PID features
        // Gains scaled for RPM: small gains since errors are in hundreds/thousands
        PID controller = new PID(0.0004, 0.00025, 0.0001).withIntegralSumMax(2000.0).withStabilityThreshold(200.0)
                .withFilter(new LowPassFilter(0.9)).withOutputLimits(-1.0, 1.0);

        double setpointRpm = 2000.0;
        double speed = 0.0;
        double dt = 0.02; // 20ms

        System.out.println("Motor Speed Control with PID");
        System.out.println("=============================");
        System.out.println("Target: 2000 RPM, Power limited to ±100%\n");
        System.out.printf("%-8s %-12s %-12s %-12s%n", "Time", "Speed(RPM)", "Error(RPM)", "Power(%)");
        System.out.println("-----------------------------------------------");

        for (int i = 0; i <= 500; i++) {
            double time = i * dt;
            double power = controller.calculate(setpointRpm, speed, dt);

            // First-order motor model: power controls target speed (0-100% → 0-3000 RPM)
            // Motor responds with time constant (tau ≈ 0.5s)
            double targetSpeed = (power + 1.0) / 2.0 * 3000.0; // Map [-1,1] to [0,3000]
            speed += (targetSpeed - speed) * 4.0 * dt; // Time constant tau = 0.25s

            if (i % 50 == 0) {
                double error = setpointRpm - speed;
                System.out.printf("%.2f     %9.1f     %9.1f     %8.1f%%%n", time, speed, error, power * 100);
            }
        }

        double finalError = Math.abs(setpointRpm - speed);
        System.out.println("\nFinal speed: " + String.format("%.1f", speed) + " RPM (target: 2000 RPM)");
        System.out.println("Final error: " + String.format("%.1f", finalError) + " RPM");
        System.out.println("\nFeatures demonstrated:");
        System.out.println("- Integral windup protection (max sum limit)");
        System.out.println("- Derivative filtering (reduces noise sensitivity)");
        System.out.println("- Stability threshold (prevents integral buildup during large transients)");
    }
}
