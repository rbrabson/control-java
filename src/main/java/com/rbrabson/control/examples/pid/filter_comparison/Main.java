package com.rbrabson.control.examples.pid.filter_comparison;

import com.rbrabson.control.filter.KalmanFilter;
import com.rbrabson.control.filter.LowPassFilter;
import com.rbrabson.control.pid.PID;

public class Main {
    public static void main(String[] args) {
        // Compare derivative filtering methods in PID controllers
        PID lowPassPID = new PID(0.5, 0.1, 0.05).withFilter(new LowPassFilter(0.5)).withOutputLimits(-100, 100);

        PID kalmanPID = new PID(0.5, 0.1, 0.05).withFilter(new KalmanFilter(0.05, 0.1, 10)).withOutputLimits(-100, 100);

        PID noFilterPID = new PID(0.5, 0.1, 0.05).withOutputLimits(-100, 100);

        double setpoint = 50.0;
        double x1 = 10.0, v1 = 0.0;
        double x2 = 10.0, v2 = 0.0;
        double x3 = 10.0, v3 = 0.0;
        double dt = 0.02;

        System.out.println("PID Filter Comparison");
        System.out.println("=====================");
        System.out.println("Comparing derivative filtering: LowPass | Kalman | None\n");
        System.out.printf("%-8s %-12s %-12s %-12s%n", "Time", "LowPass", "Kalman", "NoFilter");
        System.out.println("-----------------------------------------------");

        for (int i = 0; i <= 300; i++) {
            double time = i * dt;
            double u1 = lowPassPID.calculate(setpoint, x1, dt);
            double u2 = kalmanPID.calculate(setpoint, x2, dt);
            double u3 = noFilterPID.calculate(setpoint, x3, dt);

            // Physics simulation: output is acceleration
            v1 += u1 * dt;
            x1 += v1 * dt;
            v2 += u2 * dt;
            x2 += v2 * dt;
            v3 += u3 * dt;
            x3 += v3 * dt;

            if (i % 30 == 0) {
                System.out.printf("%.2f     %8.3f     %8.3f     %8.3f%n", time, x1, x2, x3);
            }
        }

        double e1 = Math.abs(setpoint - x1);
        double e2 = Math.abs(setpoint - x2);
        double e3 = Math.abs(setpoint - x3);

        System.out.println("\nFinal Errors:");
        System.out.printf("LowPass: %.3f | Kalman: %.3f | NoFilter: %.3f%n", e1, e2, e3);
        System.out.println("\nKey observations:");
        System.out.println("- All three approaches reach similar final accuracy");
        System.out.println("- Filtering reduces derivative noise sensitivity (important with real sensors)");
        System.out.println("- Kalman filter adapts but requires more tuning than simple low-pass");
    }
}
