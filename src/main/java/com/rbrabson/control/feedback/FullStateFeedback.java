package com.rbrabson.control.feedback;

import java.util.Arrays;

/**
 * A full state feedback controller that calculates the control input based on
 * the error between the setpoint and the measurement, multiplied by a gain
 * vector. This is a general implementation that can be used for various control
 * applications where the state is fully observable.
 */
public class FullStateFeedback {
    private final double[] gain;

    /**
     * Creates a new FullStateFeedback controller with the specified gain vector.
     * 
     * @param gain The gain vector to use for the controller. The length of the gain
     *             vector should match the length of the state vector (setpoint and
     *             measurement).
     */
    public FullStateFeedback(double[] gain) {
        this.gain = Arrays.copyOf(gain, gain.length);
    }

    /**
     * Calculates the control input based on the setpoint and measurement. The
     * control input is computed as the dot product of the error vector (setpoint -
     * measurement) and the gain vector.
     *
     * @param setpoint    The desired state vector that the controller should
     *                    achieve.
     * @param measurement The current state vector measured from the system.
     * @return The control input calculated by the full state feedback controller.
     */
    public double calculate(double[] setpoint, double[] measurement) {
        double[] errorVec = minus(setpoint, measurement);
        return product(errorVec, gain);
    }

    /**
     * Subtracts two vectors element-wise and returns the result. Both vectors must
     * be of the same length.
     *
     * @param a The first vector.
     * @param b The second vector.
     * @return A new vector that is the element-wise difference of the two input
     *         vectors (a - b).
     */

    private static double[] minus(double[] a, double[] b) {
        if (a.length != b.length) {
            throw new IllegalArgumentException("vectors must be of same length");
        }
        double[] result = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            result[i] = a[i] - b[i];
        }
        return result;
    }

    /**
     * Calculates the dot product of two vectors. Both vectors must be of the same
     * length.
     * 
     * @param a The first vector.
     * @param b The second vector.
     * @return The dot product of the two vectors, calculated as the sum of the
     *         products of corresponding elements (a[0]*b[0] + a[1]*b[1] + ... +
     *         a[n]*b[n]).
     */
    private static double product(double[] a, double[] b) {
        if (a.length != b.length) {
            throw new IllegalArgumentException("vectors must be of same length");
        }
        double sum = 0.0;
        for (int i = 0; i < a.length; i++) {
            sum += a[i] * b[i];
        }
        return sum;
    }
}