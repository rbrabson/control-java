package control.feedback;

import java.util.Arrays;

public class FullStateFeedback {
    private final double[] gain;

    public FullStateFeedback(double[] gain) {
        this.gain = Arrays.copyOf(gain, gain.length);
    }

    public double calculate(double[] setpoint, double[] measurement) {
        double[] errorVec = minus(setpoint, measurement);
        return product(errorVec, gain);
    }

    private static double[] minus(double[] a, double[] b) {
        if (a.length != b.length) {
            throw new IllegalArgumentException(FeedbackErrors.VECTORS_MUST_BE_SAME_LENGTH);
        }
        double[] result = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            result[i] = a[i] - b[i];
        }
        return result;
    }

    private static double product(double[] a, double[] b) {
        if (a.length != b.length) {
            throw new IllegalArgumentException(FeedbackErrors.VECTORS_MUST_BE_SAME_LENGTH);
        }
        double sum = 0.0;
        for (int i = 0; i < a.length; i++) {
            sum += a[i] * b[i];
        }
        return sum;
    }
}
