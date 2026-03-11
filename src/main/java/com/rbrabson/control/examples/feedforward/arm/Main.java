package com.rbrabson.control.examples.feedforward.arm;

import com.rbrabson.control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        // Arm feedforward with cosine compensation for gravity (kg = 2.5 N·m)
        // The cosine gain compensates for gravity's varying effect at different angles
        FeedForward armWithGravity = new FeedForward(0.0, 1.0, 0.2).withCosineGain(2.5);
        FeedForward armNoGravity = new FeedForward(0.0, 1.0, 0.2);

        System.out.println("Robotic Arm Feedforward Control");
        System.out.println("================================");
        System.out.println("Comparing arm control with and without gravity compensation\n");
        System.out.printf("%-10s %-12s %-15s %-15s %-15s%n", "Angle", "Angle(deg)", "With Gravity", "Without Gravity",
                "Difference");
        System.out.println("-----------------------------------------------------------------------");

        // Test various arm angles from 0° (horizontal) to 180° (straight down)
        double velocity = 1.5;
        double acceleration = 0.5;

        for (double angle = 0; angle <= Math.PI; angle += Math.PI / 12) {
            double withGrav = armWithGravity.calculate(angle, velocity, acceleration);
            double noGrav = armNoGravity.calculate(angle, velocity, acceleration);
            double diff = withGrav - noGrav;
            double degrees = Math.toDegrees(angle);

            System.out.printf("%.3f     %6.1f°      %10.3f      %10.3f      %10.3f%n", angle, degrees, withGrav, noGrav,
                    diff);
        }

        System.out.println("\nNote: Cosine compensation is highest at 0° (horizontal) and zero at 90° (vertical)");
    }
}
