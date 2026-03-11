package control.examples.interplut.adaptive_pid;

import control.interplut.InterpLUT;
import control.pid.PID;

public class Main {
    public static void main(String[] args) {
        // Adaptive PID: Kp changes based on error magnitude
        // Larger errors use higher gain for faster response
        // Smaller errors use lower gain for precise settling
        InterpLUT kpLut = new InterpLUT().withPoint(0, 0.6) // Small error: gentle control
                .withPoint(20, 0.8).withPoint(50, 1.2).withPoint(100, 1.6) // Large error: aggressive control
                .build();

        System.out.println("Adaptive PID Control");
        System.out.println("====================");
        System.out.println("Kp adapts based on error magnitude\n");
        System.out.printf("%-8s %-12s %-12s %-12s %-12s%n", "Step", "Position", "Error", "Adaptive Kp", "Output");
        System.out.println("-------------------------------------------------------");

        double setpoint = 100.0;
        double state = 10.0;
        double dt = 0.1;

        for (int i = 0; i <= 50; i++) {
            double error = setpoint - state;
            double errorMagnitude = Math.abs(error);

            // Adapt Kp based on error
            double adaptiveKp = kpLut.get(errorMagnitude);
            PID controller = new PID(adaptiveKp, 0.05, 0.02).withOutputLimits(-50, 50);

            double output = controller.calculate(setpoint, state);
            state += output * dt;

            if (i % 5 == 0) {
                System.out.printf("%4d     %8.2f     %8.2f     %8.3f     %8.3f%n", i, state, error, adaptiveKp, output);
            }
        }

        System.out.println("\nBenefits of adaptive Kp:");
        System.out.println("- Fast initial response when error is large (high Kp)");
        System.out.println("- Smooth settling when near target (low Kp)");
        System.out.println("- Reduces overshoot while maintaining speed");
    }
}
