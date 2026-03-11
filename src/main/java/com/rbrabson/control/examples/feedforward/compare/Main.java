package com.rbrabson.control.examples.feedforward.compare;

import com.rbrabson.control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        // Compare three different feedforward configurations
        FeedForward basic = new FeedForward(0.0, 1.0, 0.2);
        FeedForward elevator = new FeedForward(0.0, 1.0, 0.2).withGravityGain(9.81);
        FeedForward arm = new FeedForward(0.0, 1.0, 0.2).withCosineGain(2.5);

        System.out.println("Feedforward Configuration Comparison");
        System.out.println("=====================================");
        System.out.println("Testing same motion with different compensation methods\n");

        double[][] tests = { { 0.0, 0.5, 0.0 }, // Low speed, no accel
                { Math.PI / 6, 1.0, 0.3 }, // 30°, moderate motion
                { Math.PI / 4, 1.5, 0.5 }, // 45°, higher motion
                { Math.PI / 2, 2.0, 0.8 } // 90°, max motion
        };

        System.out.printf("%-10s %-12s %-10s %-12s %-12s %-12s%n", "Angle", "Velocity", "Accel", "Basic", "Elevator",
                "Arm");
        System.out.println("----------------------------------------------------------------");

        for (double[] test : tests) {
            double position = test[0];
            double velocity = test[1];
            double accel = test[2];

            double basicOut = basic.calculate(position, velocity, accel);
            double elevatorOut = elevator.calculate(position, velocity, accel);
            double armOut = arm.calculate(position, velocity, accel);

            System.out.printf("%.2f rad   %8.1f     %8.1f   %8.3f     %8.3f     %8.3f%n", position, velocity, accel,
                    basicOut, elevatorOut, armOut);
        }

        System.out.println("\nKey Differences:");
        System.out.println("- Basic: Only velocity + acceleration (no gravity)");
        System.out.println("- Elevator: Adds constant gravity compensation (position-independent)");
        System.out.println("- Arm: Adds cosine compensation (varies with angle)");
    }
}
