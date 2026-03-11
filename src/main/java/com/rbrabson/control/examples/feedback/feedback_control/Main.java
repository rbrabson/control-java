package com.rbrabson.control.examples.feedback.feedback_control;

import com.rbrabson.control.feedback.FullStateFeedback;

public class Main {
    public static void main(String[] args) {
        System.out.println("Full-State Feedback Control");
        System.out.println("===========================");
        System.out.println("Controls both position and velocity simultaneously");
        System.out.println("Using gain matrix K = [position_gain, velocity_gain]\n");

        // Create controller with gains: K = [position_gain, velocity_gain]
        // Higher velocity gain provides more damping to reduce oscillation
        FullStateFeedback controller = new FullStateFeedback(new double[] { 5.0, 5.0 });

        double[] setpoint = { 10.0, 0.0 }; // Target: position=10, velocity=0
        double[] state = { 6.5, 0.8 }; // Initial: position=6.5, velocity=0.8

        double dt = 0.05; // Time step
        int iterations = 100;

        System.out.printf("%-6s %-10s %-10s %-10s %-10s%n", "Time", "Position", "Velocity", "PosError", "Output");
        System.out.println("--------------------------------------------------");

        for (int i = 0; i <= iterations; i++) {
            double t = i * dt;

            // Calculate control output
            double output = controller.calculate(setpoint, state);

            // Simulate physics: output is acceleration
            state[1] += output * dt; // velocity += acceleration * dt
            state[0] += state[1] * dt; // position += velocity * dt

            // Print every 10 iterations
            if (i % 10 == 0) {
                double posError = setpoint[0] - state[0];
                System.out.printf("%.2f   %8.3f   %8.3f   %8.3f   %8.3f%n", t, state[0], state[1], posError, output);
            }
        }

        System.out.println("\nFinal Results:");
        System.out.printf("Target position: %.1f, Final position: %.3f\n", setpoint[0], state[0]);
        System.out.printf("Target velocity: %.1f, Final velocity: %.3f\n", setpoint[1], state[1]);
        System.out.println("\nKey advantages:");
        System.out.println("- Controls multiple state variables simultaneously");
        System.out.println("- Better damping than position-only control");
        System.out.println("- Velocity feedback prevents overshoot");
    }
}
