package com.rbrabson.control.examples.pid.dampening;

import com.rbrabson.control.filter.LowPassFilter;
import com.rbrabson.control.pid.PID;

public class Main {
    public static void main(String[] args) {
        // Compare three PID configurations to demonstrate dampening effects
        PID basic = new PID(2.0, 0.5, 0.8).withOutputLimits(-50, 50);
        PID filtered = new PID(2.0, 0.5, 0.8).withFilter(new LowPassFilter(0.6)).withOutputLimits(-50, 50);
        PID damped = new PID(2.0, 0.5, 0.8).withFilter(new LowPassFilter(0.6)).withStabilityThreshold(3.0)
                .withOutputLimits(-50, 50);

        double target = 100.0;
        double x1 = 20.0, v1 = 0.0;
        double x2 = 20.0, v2 = 0.0;
        double x3 = 20.0, v3 = 0.0;
        double dt = 0.02;

        System.out.println("PID Dampening Comparison");
        System.out.println("========================");
        System.out.println("Comparing: Basic | Filtered | Filtered+Stability\n");
        System.out.printf("%-8s %-12s %-12s %-12s%n", "Time", "Basic", "Filtered", "Damped");
        System.out.println("-----------------------------------------------");

        for (int i = 0; i <= 200; i++) {
            double time = i * dt;
            double u1 = basic.calculate(target, x1, dt);
            double u2 = filtered.calculate(target, x2, dt);
            double u3 = damped.calculate(target, x3, dt);

            // Physics simulation: output is acceleration
            v1 += u1 * dt;
            x1 += v1 * dt;
            v2 += u2 * dt;
            x2 += v2 * dt;
            v3 += u3 * dt;
            x3 += v3 * dt;

            if (i % 20 == 0) {
                System.out.printf("%.2f     %8.2f     %8.2f     %8.2f%n", time, x1, x2, x3);
            }
        }

        double e1 = Math.abs(target - x1);
        double e2 = Math.abs(target - x2);
        double e3 = Math.abs(target - x3);

        System.out.println("\nFinal Errors:");
        System.out.printf("Basic: %.2f | Filtered: %.2f | Damped: %.2f%n", e1, e2, e3);
        System.out.println("\nKey observations:");
        System.out.println("- Filtered derivative: Similar performance to basic, less noise sensitive");
        System.out.println("- Stability threshold: Prevents integral wind-up during large errors");
        System.out.println("  (trades slower convergence for better transient behavior)");
    }
}
