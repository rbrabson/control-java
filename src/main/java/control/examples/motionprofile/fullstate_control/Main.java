package control.examples.motionprofile.fullstate_control;

import control.feedback.FullStateFeedback;
import control.motionprofile.Constraints;
import control.motionprofile.MotionProfile;
import control.motionprofile.State;

public class Main {
    public static void main(String[] args) {
        System.out.println("Motion Profile + Full-State Feedback");
        System.out.println("====================================");
        System.out.println("Tracking a motion profile using full-state feedback");
        System.out.println("with feedforward from profile acceleration\n");

        MotionProfile profile = new MotionProfile(new Constraints(3.0, 2.0), new State(0.0, 0.0, 0.0, 0.0),
                new State(4.0, 0.0, 0.0, 0.0));

        // Increase gains for better tracking: K = [position_gain, velocity_gain]
        FullStateFeedback fsf = new FullStateFeedback(new double[] { 10.0, 5.0 });
        double[] measured = { 0.0, 0.0 };

        // Use smaller time step for numerical stability
        double dt = 0.02;
        int printEvery = 5; // Print every 5 iterations (0.1 seconds)
        int iteration = 0;

        double maxPosError = 0.0;
        double finalPosError = 0.0;

        System.out.printf("%-6s %-10s %-10s %-8s %-10s %-10s %-8s%n", "Time", "DesPos", "MeasPos", "PosErr", "DesVel",
                "MeasVel", "Output");
        System.out.println("------------------------------------------------------------------");

        for (double t = 0.0; t <= profile.totalTime(); t += dt) {
            State desired = profile.calculate(t);
            double output = fsf.calculate(new double[] { desired.position, desired.velocity }, measured);

            // Add feedforward from motion profile acceleration for better tracking
            output += desired.acceleration;

            // Simple physics simulation: output is acceleration
            measured[1] += output * dt; // velocity += acceleration * dt
            measured[0] += measured[1] * dt; // position += velocity * dt

            // Track errors
            double posError = desired.position - measured[0];
            maxPosError = Math.max(maxPosError, Math.abs(posError));
            finalPosError = posError;

            // Print at intervals for readability
            if (iteration % printEvery == 0) {
                System.out.printf("%.2f   %7.3f    %7.3f    %6.3f   %7.3f    %7.3f    %6.3f%n", t, desired.position,
                        measured[0], posError, desired.velocity, measured[1], output);
            }
            iteration++;
        }

        System.out.println("\nTracking Performance:");
        System.out.printf("Target: 4.0 units, Final position: %.3f units\n", measured[0]);
        System.out.printf("Max position error: %.3f units\n", maxPosError);
        System.out.printf("Final position error: %.3f units\n", Math.abs(finalPosError));
        System.out.println("\nControl strategy:");
        System.out.println("- Feedforward: Uses profile acceleration to anticipate motion");
        System.out.println("- Feedback: Corrects position and velocity errors");
        System.out.println("- Result: Tight tracking with minimal steady-state error");
    }
}
