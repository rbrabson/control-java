package control.examples.feedback.feedback_control;

import control.feedback.FullStateFeedback;

public class Main {
    public static void main(String[] args) {
        FullStateFeedback controller = new FullStateFeedback(new double[] { 1.5, 0.3 });

        double[] setpoint = { 10.0, 0.0 };
        double[] measurement = { 6.5, 0.8 };

        double output = controller.calculate(setpoint, measurement);
        System.out.printf("Setpoint=[%.1f, %.1f], Measurement=[%.1f, %.1f], Output=%.3f%n", setpoint[0], setpoint[1],
                measurement[0], measurement[1], output);
    }
}
