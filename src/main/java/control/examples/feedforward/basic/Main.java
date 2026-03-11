package control.examples.feedforward.basic;

import control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        FeedForward ff = new FeedForward(0.0, 1.2, 0.3);

        System.out.println("Basic Feedforward Control");
        System.out.println("==========================");
        System.out.println("Formula: output = ks + kv*velocity + ka*acceleration");
        System.out.println("Gains: ks=0.0, kv=1.2, ka=0.3\n");
        System.out.printf("%-12s %-12s %-12s %-12s%n", "Velocity", "Accel", "Output", "Breakdown");
        System.out.println("-------------------------------------------------------");

        // Test different velocity and acceleration combinations
        double[][] tests = { { 0.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 }, { 2.0, 1.0 }, { 5.0, 2.0 }, { 10.0, 5.0 } };

        for (double[] test : tests) {
            double velocity = test[0];
            double accel = test[1];
            double output = ff.calculate(0.0, velocity, accel);
            String breakdown = String.format("%.1f+%.1f", velocity * 1.2, accel * 0.3);
            System.out.printf("%8.1f     %8.1f     %8.3f     %s%n", velocity, accel, output, breakdown);
        }

        System.out.println("\nNote: Higher velocity needs more power to overcome drag/friction");
        System.out.println("      Acceleration needs additional power to change speed quickly");
    }
}
