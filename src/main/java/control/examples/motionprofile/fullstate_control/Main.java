package control.examples.motionprofile.fullstate_control;

import control.feedback.FullStateFeedback;
import control.motionprofile.Constraints;
import control.motionprofile.MotionProfile;
import control.motionprofile.State;

public class Main {
    public static void main(String[] args) {
        MotionProfile profile = new MotionProfile(new Constraints(3.0, 2.0), new State(0.0, 0.0, 0.0, 0.0),
                new State(4.0, 0.0, 0.0, 0.0));

        // Increase gains for better tracking: K = [position_gain, velocity_gain]
        FullStateFeedback fsf = new FullStateFeedback(new double[] { 10.0, 5.0 });
        double[] measured = { 0.0, 0.0 };

        // Use smaller time step for numerical stability
        double dt = 0.02;
        int printEvery = 5; // Print every 5 iterations (0.1 seconds)
        int iteration = 0;

        System.out.printf("%-6s %-10s %-10s %-8s %-10s %-10s %-8s%n", 
            "Time", "DesPos", "MeasPos", "PosErr", "DesVel", "MeasVel", "Output");
        System.out.println("------------------------------------------------------------------");

        for (double t = 0.0; t <= profile.totalTime(); t += dt) {
            State desired = profile.calculate(t);
            double output = fsf.calculate(new double[] { desired.position, desired.velocity }, measured);
            
            // Add feedforward from motion profile acceleration for better tracking
            output += desired.acceleration;

            // Simple physics simulation: output is acceleration
            measured[1] += output * dt; // velocity += acceleration * dt
            measured[0] += measured[1] * dt; // position += velocity * dt

            // Print at intervals for readability
            if (iteration % printEvery == 0) {
                double posError = desired.position - measured[0];
                System.out.printf("%.2f   %7.3f    %7.3f    %6.3f   %7.3f    %7.3f    %6.3f%n",
                    t, desired.position, measured[0], posError, 
                    desired.velocity, measured[1], output);
            }
            iteration++;
        }
    }
}
