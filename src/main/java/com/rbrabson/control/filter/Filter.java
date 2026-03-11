package com.rbrabson.control.filter;

/**
 * A simple interface for a filter that can be used to estimate a value based on
 * measurements. The filter can be reset to clear its internal state, and it
 * provides a method to get the current gain of the filter, which can be useful
 * for tuning and debugging purposes.
 */
public interface Filter {

    /**
     * Estimates the value based on the given measurement.
     *
     * @param measurement The current measurement to be processed by the filter.
     * @return The estimated value based on the measurement and the filter's
     *         internal state.
     */
    double estimate(double measurement);

    /**
     * Resets the filter's internal state, clearing any accumulated information and
     * returning it to its initial state. This can be useful when starting a new
     * estimation process or when the filter's state has become unreliable due to a
     * significant change in the system being measured.
     */
    void reset();

    /**
     * Returns the current gain of the filter, which represents how much the filter
     * is relying on the measurements versus its internal state. A higher gain
     * indicates that the filter is more responsive to new measurements, while a
     * lower gain indicates that the filter is relying more on its internal state
     * and less on new measurements. This can be useful for tuning the filter and
     * understanding its behavior in different conditions.
     *
     * @return The current gain of the filter, indicating its responsiveness to new
     *         measurements.
     */
    double getAlpha();
}
