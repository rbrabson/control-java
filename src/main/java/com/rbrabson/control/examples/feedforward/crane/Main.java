package com.rbrabson.control.examples.feedforward.crane;

import com.rbrabson.control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        // Crane with both gravity (vertical lift) and cosine (boom angle) compensation
        // kg = constant gravity for lifting load
        // kc = cosine compensation for boom angle (horizontal position affects torque)
        FeedForward craneFF = new FeedForward(0.0, 1.1, 0.25).withGravityGain(15.7).withCosineGain(8.2);

        System.out.println("Crane Feedforward Control");
        System.out.println("==========================");
        System.out.println("Combines gravity (vertical lift) + cosine (boom angle) compensation\n");
        System.out.printf("%-10s %-12s %-12s %-12s %-12s%n", "Angle", "Angle(deg)", "Velocity", "Accel", "Output");
        System.out.println("----------------------------------------------------------");

        // Test different boom angles and motions
        double[][] tests = { { 0.0, 0.5, 0.0 }, // Horizontal, moving
                { Math.PI / 6, 0.5, 0.0 }, // 30°, moving
                { Math.PI / 4, 0.8, 0.1 }, // 45°, moving with accel
                { Math.PI / 3, 1.0, 0.2 }, // 60°, moving faster
                { Math.PI / 2, 1.0, 0.0 }, // Vertical, moving
                { 2 * Math.PI / 3, 0.5, 0.0 } // 120°, moving
        };

        for (double[] test : tests) {
            double angle = test[0];
            double velocity = test[1];
            double accel = test[2];
            double output = craneFF.calculate(angle, velocity, accel);
            double degrees = Math.toDegrees(angle);

            System.out.printf("%.3f     %6.1f°      %8.1f     %8.1f     %8.3f%n", angle, degrees, velocity, accel,
                    output);
        }

        System.out.println("\nNote: Output varies with boom angle due to changing torque requirements");
        System.out.println("      Horizontal (0°) requires max compensation, vertical (90°) needs least");
    }
}
