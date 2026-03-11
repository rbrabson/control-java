package control.examples.feedforward.elevator;

import control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        // Elevator feedforward with gravity compensation (kg = 9.81 m/s²)
        FeedForward elevatorWithGravity = new FeedForward(0.0, 0.9, 0.2, FeedForward.withGravityGain(9.81));
        FeedForward elevatorNoGravity = new FeedForward(0.0, 0.9, 0.2);

        System.out.println("Elevator Feedforward Control");
        System.out.println("=============================");
        System.out.println("Gravity compensation is constant (doesn't depend on position)");
        System.out.println("This represents the motor power needed to hold the elevator stationary\n");
        System.out.printf("%-12s %-12s %-15s %-15s %-15s%n", "Velocity", "Accel", "With Gravity", "Without Gravity",
                "Difference");
        System.out.println("--------------------------------------------------------------------");

        // Test different motion scenarios
        double[][] tests = { { 0.0, 0.0 }, // Stationary
                { 1.0, 0.0 }, // Constant speed up
                { -1.0, 0.0 }, // Constant speed down
                { 0.0, 2.0 }, // Accelerating up
                { 0.0, -2.0 }, // Accelerating down
                { 2.0, 1.0 }, // Moving and accelerating up
                { -2.0, -1.0 } // Moving and accelerating down
        };

        for (double[] test : tests) {
            double velocity = test[0];
            double accel = test[1];
            double withGrav = elevatorWithGravity.calculate(0.0, velocity, accel);
            double noGrav = elevatorNoGravity.calculate(0.0, velocity, accel);
            double diff = withGrav - noGrav;

            System.out.printf("%8.1f     %8.1f     %11.3f     %11.3f     %11.3f%n", velocity, accel, withGrav, noGrav,
                    diff);
        }

        System.out.println("\nNote: Gravity compensation adds constant ~9.81 to counter weight");
        System.out.println("      Negative velocity/accel = moving/accelerating downward");
    }
}
