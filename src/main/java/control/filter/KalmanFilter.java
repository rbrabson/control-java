package control.filter;

import java.util.List;

/**
 * A Kalman filter implementation for estimating the state of a system based on
 * noisy measurements. This implementation uses a simple linear model and
 * assumes constant process and measurement noise.
 */
public class KalmanFilter implements Filter {
    private final double q;
    private final double r;
    private final int n;
    private double p;
    private double k;
    private double x;
    private SizedStack<Double> estimates;
    private LinearRegression regression;

    /**
     * Creates a new Kalman filter with the specified process noise covariance (q),
     * measurement noise covariance (r), and stack size (n).
     *
     * @param q the process noise covariance, which represents the uncertainty in
     *          the system's dynamics
     * @param r the measurement noise covariance, which represents the uncertainty
     *          in the measurements
     * @param n the size of the stack used to store recent estimates for regression,
     *          which helps in predicting future values
     */
    public KalmanFilter(double q, double r, int n) {
        if (n <= 0) {
            throw new IllegalArgumentException("stack size must be positive");
        }
        if (q < 0 || r < 0) {
            throw new IllegalArgumentException("covariance values must be non-negative");
        }

        this.q = q;
        this.r = r;
        this.n = n;
        this.p = 1.0;
        this.k = 0.0;
        this.x = 0.0;
        this.estimates = new SizedStack<>(n);
        reset();

        for (int i = 0; i < n; i++) {
            this.estimates.push(0.0);
        }

        this.regression = new LinearRegression(toPrimitive(estimates));
        findK();
    }

    /**
     * Sets the initial state estimate (x) for the Kalman filter. This can be used
     * to provide a better starting point for the filter, especially if there is
     * prior knowledge about the system's state.
     *
     * @param x the initial state estimate to set for the Kalman filter
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Returns the current state estimate (x) of the Kalman filter.
     *
     * @return the current state estimate of the Kalman filter
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the current Kalman gain (k) of the filter, which represents how much
     * the filter trusts the measurements compared to the predictions.
     *
     * @return the current Kalman gain of the filter
     */
    public double getK() {
        return k;
    }

    /**
     * Returns the current error covariance (p) of the filter, which represents the
     * uncertainty in the state estimate.
     *
     * @return the current error covariance of the filter
     */
    public double getP() {
        return p;
    }

    /**
     * Returns the process noise covariance (q) of the filter, which represents the
     * uncertainty in the system's dynamics.
     *
     * @return the process noise covariance of the filter
     */
    @Override
    public double getAlpha() {
        return k;
    }

    /**
     * Estimates the current state based on the provided measurement. The method
     * first runs a linear regression on the recent estimates to predict the next
     * value, then updates the state estimate (x) using the Kalman gain (k) and the
     * difference between the measurement and the current estimate.
     *
     * @param measurement The current measurement to be processed by the filter.
     * @return The updated state estimate after processing the measurement.
     */
    @Override
    public double estimate(double measurement) {
        regression.runLeastSquares();
        double prediction = regression.predictNextValue();

        Double latest = estimates.peek();
        double lastEstimate = latest == null ? 0.0 : latest;

        x += prediction - lastEstimate;
        x += k * (measurement - x);

        estimates.push(x);
        regression.updateData(toPrimitive(estimates));

        return x;
    }

    /**
     * Finds the optimal Kalman gain (k) by iteratively solving the Discrete
     * Algebraic Riccati Equation (DARE).
     */
    private void findK() {
        for (int i = 0; i < 2000; i++) {
            solveDARE();
        }
    }

    /**
     * Solves the Discrete Algebraic Riccati Equation (DARE) to update the error
     * covariance (p) and Kalman gain (k) based on the process noise covariance (q)
     * and measurement noise covariance (r). This method is called iteratively to
     * find the optimal gain for the filter.
     */
    private void solveDARE() {
        p += q;
        k = p / (p + r);
        p = (1 - k) * p;
    }

    /**
     * Resets the Kalman filter to its initial state while preserving the current
     * values of the Kalman gain (k) and error covariance (p).
     */
    @Override
    public void reset() {
        double convergedP = p;
        double convergedK = k;

        x = 0.0;
        estimates = new SizedStack<>(n);
        for (int i = 0; i < n; i++) {
            estimates.push(0.0);
        }

        regression = new LinearRegression(toPrimitive(estimates));

        p = convergedP;
        k = convergedK;
    }

    /**
     * Converts a SizedStack of Double objects to a primitive double array. This is
     * necessary for the LinearRegression class, which operates on primitive arrays
     * for efficiency.
     *
     * @param stack the SizedStack of Double objects to be converted to a primitive
     *              double array
     * @return a primitive double array containing the values from the SizedStack
     */

    private static double[] toPrimitive(SizedStack<Double> stack) {
        List<Double> values = stack.toList();
        double[] out = new double[values.size()];
        for (int i = 0; i < values.size(); i++) {
            out[i] = values.get(i);
        }
        return out;
    }
}
