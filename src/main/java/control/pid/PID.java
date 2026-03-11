package control.pid;

import control.filter.Filter;

/**
 * A PID (Proportional-Integral-Derivative) controller implementation that
 * calculates an output based on the error between a reference value and a
 * current state. The controller can be configured with various options, such as
 * feedforward, integral reset on zero crossing, stability threshold for the
 * derivative term, maximum integral sum, output limits, and a filter for
 * smoothing the derivative term. The PID controller is designed to be flexible
 * and adaptable to different control scenarios, allowing for fine-tuning of its
 * behavior through the use of fluent methods that return copies.
 */
public class PID {
    private final double kp;
    private final double ki;
    private double kd;

    private double feedForward;
    private boolean integralResetOnZeroCross;
    private double stabilityThreshold;
    private double integralSumMax;
    private double outputMin;
    private double outputMax;
    private Filter filter;

    private double integral;
    private double lastReference;
    private double lastError;
    private long prevTimeNanos;
    private boolean initialized;

    /**
     * Creates a PID controller with the specified gains.
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     */
    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.outputMin = Double.NEGATIVE_INFINITY;
        this.outputMax = Double.POSITIVE_INFINITY;
        this.initialized = false;
        this.integralResetOnZeroCross = false;
        this.filter = null;
        this.stabilityThreshold = Double.NaN;
        this.integralSumMax = Double.NaN;
    }

    /**
     * Copy constructor for creating a new PID controller with the same
     * configuration.
     *
     * @param other The PID controller to copy.
     */
    private PID(PID other) {
        this.kp = other.kp;
        this.ki = other.ki;
        this.kd = other.kd;
        this.feedForward = other.feedForward;
        this.integralResetOnZeroCross = other.integralResetOnZeroCross;
        this.stabilityThreshold = other.stabilityThreshold;
        this.integralSumMax = other.integralSumMax;
        this.outputMin = other.outputMin;
        this.outputMax = other.outputMax;
        this.filter = other.filter;
        this.integral = other.integral;
        this.lastReference = other.lastReference;
        this.lastError = other.lastError;
        this.prevTimeNanos = other.prevTimeNanos;
        this.initialized = other.initialized;
    }

    /**
     * Creates a copy of this PID controller with a feedforward term.
     *
     * @param feedForward The feedforward term to be added to the output of the PID
     *                    controller.
     * @return A new PID controller with the feedforward term set.
     */
    public PID withFeedForward(double feedForward) {
        PID copy = new PID(this);
        copy.feedForward = feedForward;
        return copy;
    }

    /**
     * Creates a copy of this PID controller with integral reset on zero crossing
     * enabled. When enabled, the integral term will be reset to zero whenever the
     * error crosses zero (i.e., when the system transitions from being above the
     * target to below the target, or vice versa).
     *
     * @return A new PID controller with integral reset on zero crossing enabled.
     */
    public PID withIntegralResetOnZeroCross() {
        PID copy = new PID(this);
        copy.integralResetOnZeroCross = true;
        return copy;
    }

    /**
     * Creates a copy of this PID controller with a stability threshold. The
     * stability threshold is used to determine when the derivative term should be
     * calculated. If the absolute value of the raw derivative exceeds the stability
     * threshold, the integral term will not be updated.
     *
     * @param threshold The stability threshold value.
     * @return A new PID controller with the stability threshold set.
     */
    public PID withStabilityThreshold(double threshold) {
        PID copy = new PID(this);
        copy.stabilityThreshold = Math.abs(threshold);
        return copy;
    }

    /**
     * Creates a copy of this PID controller with a maximum integral sum. If the
     * integral sum exceeds this value, it will be clamped to the maximum.
     *
     * @param maxSum The maximum absolute value for the integral sum.
     * @return A new PID controller with the integral sum max set.
     */
    public PID withIntegralSumMax(double maxSum) {
        PID copy = new PID(this);
        copy.integralSumMax = Math.abs(maxSum);
        return copy;
    }

    /**
     * Creates a copy of this PID controller with a filter for the derivative term.
     * The filter will be applied to the raw derivative value before it is
     * multiplied by the derivative gain (kd).
     *
     * @param filter The filter to be used for smoothing the derivative term.
     * @return A new PID controller with the filter set.
     */
    public PID withFilter(Filter filter) {
        PID copy = new PID(this);
        copy.filter = filter;
        return copy;
    }

    /**
     * Creates a copy of this PID controller with output limits. The output of the
     * PID controller will be clamped to the specified minimum and maximum values.
     *
     * @param min The minimum output value for the PID controller.
     * @param max The maximum output value for the PID controller.
     * @return A new PID controller with the output limits set.
     */
    public PID withOutputLimits(double min, double max) {
        PID copy = new PID(this);
        if (min <= max) {
            copy.outputMin = min;
            copy.outputMax = max;
        }
        return copy;
    }

    /**
     * Creates a copy of this PID controller with PID gains based on a damped
     * spring-mass-damper system model. The gains are calculated based on the
     * specified spring stiffness (ka), mass (kv), and percent overshoot (po).
     *
     * @param ka The spring stiffness, which represents the proportional gain (kp)
     * @param kv The mass, which represents the derivative gain (kd)
     * @param po The percent overshoot, used to calculate the damping ratio (zeta)
     * @return A new PID controller with the derivative gain set based on dampening.
     */
    public PID withDampening(double ka, double kv, double po) {
        PID copy = new PID(this);
        if (copy.kp < kv * kv / 4 * ka) {
            return copy;
        }

        if (po == 0) {
            copy.kd = 2 * Math.sqrt(ka * kv) - ka;
        } else {
            double boundedPo = Math.max(po / 100.0, 0.01);
            double poLog = Math.log(boundedPo);
            double zeta = -poLog / Math.sqrt(Math.PI * Math.PI + poLog * poLog);
            copy.kd = 2 * zeta * Math.sqrt(ka * kv) - kv;
        }
        return copy;
    }

    /**
     * Calculates the PID output based on the given reference and current state. The
     * method computes the proportional, integral, and derivative terms based on the
     * error between the reference and state, and applies any configured
     * feedforward, stability threshold, integral sum limits, and output limits.
     * This method uses real elapsed time (via System.nanoTime()) to calculate dt.
     *
     * @param reference The target reference value that the PID controller is trying
     *                  to achieve.
     * @param state     The current state value that the PID controller is using to
     *                  calculate the error and output.
     * @return The calculated output from the PID controller, which is the sum of
     *         the proportional, integral, and derivative terms, plus any
     *         feedforward, and clamped to the configured output limits.
     */
    public double calculate(double reference, double state) {
        long now = System.nanoTime();

        if (!initialized) {
            prevTimeNanos = now;
        }

        double dt = (now - prevTimeNanos) / 1_000_000_000.0;
        prevTimeNanos = now;

        return calculate(reference, state, dt);
    }

    /**
     * Calculates the PID output based on the given reference, current state, and
     * time delta. This overload is intended for simulation use where you want to
     * specify the time step explicitly rather than relying on real elapsed time.
     * The method computes the proportional, integral, and derivative terms based on
     * the error between the reference and state, and applies any configured
     * feedforward, stability threshold, integral sum limits, and output limits.
     *
     * @param reference The target reference value that the PID controller is trying
     *                  to achieve.
     * @param state     The current state value that the PID controller is using to
     *                  calculate the error and output.
     * @param dt        The time delta in seconds to use for integral and derivative
     *                  calculations. This allows for simulation with a fixed time
     *                  step independent of real elapsed time.
     * @return The calculated output from the PID controller, which is the sum of
     *         the proportional, integral, and derivative terms, plus any
     *         feedforward, and clamped to the configured output limits.
     */
    public double calculate(double reference, double state, double dt) {
        double error = reference - state;

        if (!initialized) {
            integral = 0;
            lastReference = reference;
            lastError = error;
            initialized = true;
        }

        if (reference != lastReference) {
            integral = 0;
            lastReference = reference;
        }

        double proportional = calculateProportional(error);
        double derivative = calculateDerivative(error, dt);
        double integralTerm = calculateIntegral(error, dt);

        double output = proportional + integralTerm + derivative + feedForward;
        double clampedOutput = clamp(output, outputMin, outputMax);

        // Anti-windup: Adjust integral if output is clamped
        if (output != clampedOutput && ki != 0) {
            integral = (clampedOutput - proportional - derivative - feedForward) / ki;
        }

        lastError = error;

        return clampedOutput;
    }

    /**
     * Calculates the proportional term of the PID controller based on the given
     * error and the proportional gain (kp).
     * 
     * @param error The current error value, which is the difference between the
     *              reference and the state.
     * @return The calculated proportional term, which is the product of the error
     *         and the proportional gain (kp).
     */
    private double calculateProportional(double error) {
        return kp * error;
    }

    /**
     * Calculates the integral term of the PID controller based on the given error,
     * time delta (dt), and the integral gain (ki).
     *
     * @param error The current error value, which is the difference between the
     *              reference and the state.
     * @param dt    The time delta in seconds since the last calculation.
     * @return The calculated integral term, which is the product of the integral
     *         gain (ki) and the accumulated integral value.
     */
    private double calculateIntegral(double error, double dt) {
        if (integralResetOnZeroCross && ((lastError > 0 && error < 0) || (lastError < 0 && error > 0))) {
            integral = 0;
        }

        double rawDerivative = calculateRawDerivative(error, dt);
        if (Double.isNaN(stabilityThreshold) || Math.abs(rawDerivative) <= stabilityThreshold) {
            integral += error * dt;

            if (!Double.isNaN(integralSumMax)) {
                integral = clamp(integral, -integralSumMax, integralSumMax);
            }
        }

        return ki * integral;
    }

    /**
     * Calculates the derivative term of the PID controller based on the given
     * error, time delta (dt), and the derivative gain (kd).
     * 
     * @param error The current error value, which is the difference between the
     *              reference and the state.
     * @param dt    The time delta in seconds since the last calculation.
     * @return The calculated derivative term, which is the product of the
     *         derivative gain (kd) and the estimated derivative of the error.
     */
    private double calculateDerivative(double error, double dt) {
        double rawDerivative = calculateRawDerivative(error, dt);
        return kd * rawDerivative;
    }

    /**
     * Calculates the raw derivative of the error based on the change in error and
     * the time delta (dt).
     * 
     * @param error The current error value, which is the difference between the
     *              reference and the state.
     * @param dt    The time delta in seconds since the last calculation.
     * @return The calculated raw derivative of the error, which is the change in
     *         error divided by the time delta (dt).
     */
    private double calculateRawDerivative(double error, double dt) {
        if (dt <= 0) {
            return 0;
        }

        double errorChange = error - lastError;
        double currentEstimate = filter != null ? filter.estimate(errorChange) : errorChange;
        return currentEstimate / dt;
    }

    /**
     * Resets the internal state of the PID controller, including the integral term,
     * last error, and initialization flag. This can be useful when the controller
     * needs to be reinitialized or when the system is being restarted. The method
     * also resets the configured filter if one is set to ensure that any internal
     * state of the filter is also cleared. After calling this method, the PID
     * controller will be in a clean state, ready to start fresh calculations on the
     * next update.
     *
     * @return The PID instance itself, allowing for method chaining when resetting
     *         and reconfiguring the controller.
     */
    public PID reset() {
        integral = 0;
        initialized = false;
        lastError = 0;
        if (filter != null) {
            filter.reset();
        }
        return this;
    }

    /**
     * Clamps a value between a minimum and maximum range.
     *
     * @param value The value to clamp.
     * @param min   The minimum allowed value.
     * @param max   The maximum allowed value.
     * @return The clamped value, constrained to be within [min, max].
     */
    private static double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }
}
