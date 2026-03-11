package control.filter;

/**
 * A simple low-pass filter that smooths out noisy measurements by blending the
 * previous estimate with the new measurement. The alpha parameter controls how
 * much weight is given to the new measurement versus the previous estimate.
 * Higher alpha = less smoothing (faster response), lower alpha = more
 * smoothing.
 */
public class LowPassFilter implements Filter {
    private double alpha;
    private double previousEstimate;
    private boolean initialized;

    /**
     * Creates a new LowPassFilter with the specified alpha smoothing factor.
     * 
     * @param alpha The smoothing factor for the filter, which must be between 0 and
     *              1 (exclusive). Higher alpha gives more weight to new
     *              measurements (less smoothing, faster response), while lower
     *              alpha gives more weight to the previous estimate (more
     *              smoothing, slower response).
     */
    public LowPassFilter(double alpha) {
        if (alpha <= 0 || alpha >= 1) {
            throw new IllegalArgumentException("alpha must be between 0 and 1 (exclusive)");
        }
        this.alpha = alpha;
        this.previousEstimate = 0.0;
        this.initialized = false;
    }

    /**
     * Estimates the current value based on the new measurement and the previous
     * estimate. If the filter has not been initialized yet, it will simply return
     * the first measurement as the initial estimate.
     *
     * @param measurement The current measurement to be processed by the filter.
     * @return The estimated value after applying the low-pass filter to the
     *         measurement.
     */
    @Override
    public double estimate(double measurement) {
        if (!initialized) {
            previousEstimate = measurement;
            initialized = true;
            return measurement;
        }

        double estimate = alpha * measurement + (1 - alpha) * previousEstimate;
        previousEstimate = estimate;
        return estimate;
    }

    /**
     * Returns the current alpha smoothing factor of the filter.
     *
     * @return The alpha value, which is between 0 and 1 (exclusive).
     */
    @Override
    public double getGain() {
        return alpha;
    }

    /**
     * Sets the alpha smoothing factor of the filter. Alpha must be between 0 and 1
     * (exclusive). Higher alpha gives more weight to new measurements (less
     * smoothing, faster response), while lower alpha gives more weight to the
     * previous estimate (more smoothing, slower response).
     *
     * @param alpha The new alpha value to set, which must be between 0 and 1
     *              (exclusive).
     */
    public void setAlpha(double alpha) {
        if (alpha <= 0 || alpha >= 1) {
            throw new IllegalArgumentException("alpha must be between 0 and 1 (exclusive)");
        }
        this.alpha = alpha;
    }

    /**
     * Resets the filter by clearing the previous estimate and marking it as
     * uninitialized. After calling this method, the next call to estimate() will
     * treat the next measurement as the initial estimate.
     */
    @Override
    public void reset() {
        previousEstimate = 0.0;
        initialized = false;
    }

    /**
     * Returns the last estimate produced by the filter. If the filter has not been
     * initialized yet, it will return 0.0.
     *
     * @return The last estimate produced by the filter, or 0.0 if the filter has
     *         not been initialized.
     */
    public double getLastEstimate() {
        return initialized ? previousEstimate : 0.0;
    }

    /**
     * Returns whether the filter has been initialized with at least one
     * measurement. If the filter has not been initialized, it will return false,
     * and the next call to estimate() will treat the next measurement
     *
     * @return true if the filter has been initialized with at least one
     *         measurement, false otherwise.
     */
    public boolean isInitialized() {
        return initialized;
    }
}
