package control.pid;

import control.filter.Filter;
import java.util.function.Consumer;

/**
 * A PID (Proportional-Integral-Derivative) controller implementation that
 * calculates an output based on the error between a reference value and a
 * current state. The controller can be configured with various options, such as
 * feedforward, integral reset on zero crossing, stability threshold for the
 * derivative term, maximum integral sum, output limits, and a filter for
 * smoothing the derivative term. The PID controller is designed to be flexible
 * and adaptable to different control scenarios, allowing for fine-tuning of its
 * behavior through the use of functional options.
 */
public class PID {
    /**
     * Functional interface for configuring optional parameters of the PID
     * controller. Each option is a lambda that accepts a PID instance and modifies
     * its configuration. This allows for flexible and readable configuration of the
     * PID controller when creating an instance, without needing to provide all
     * parameters in the constructor.
     */
    public interface Option extends Consumer<PID> {
    }

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
     * Creates a PID controller with the specified gains and optional configuration
     * parameters.
     *
     * @param kp      The proportional gain.
     * @param ki      The integral gain.
     * @param kd      The derivative gain.
     * @param options Optional configuration parameters for the PID controller.
     *                These can be used to override the default values of the PID
     *                controller.
     */
    public PID(double kp, double ki, double kd, Option... options) {
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

        for (Option option : options) {
            option.accept(this);
        }
    }

    /**
     * Creates an Option that sets a feedforward term for the PID controller.
     *
     * @param feedForward The feedforward term to be added to the output of the PID
     *                    controller.
     * @return An Option that sets the feedForward term of the PID controller to the
     *         specified value.
     */

    public static Option withFeedForward(double feedForward) {
        return p -> p.feedForward = feedForward;
    }

    /**
     * Creates an Option that enables integral reset on zero crossing. When enabled,
     * the integral term will be reset to zero whenever the error crosses zero
     * (i.e., when the system transitions from being above the target to below the
     * target, or vice versa). This can help prevent integral windup and improve
     * stability in certain scenarios where the system may oscillate around the
     * target.
     *
     * @return An Option that sets the integralResetOnZeroCross flag of the PID
     *         controller to true, which will reset the integral term whenever the
     *         error crosses zero
     */

    public static Option withIntegralResetOnZeroCross() {
        return p -> p.integralResetOnZeroCross = true;
    }

    /**
     * Creates an Option that sets the stability threshold for the PID controller.
     * The stability threshold is used to determine when the derivative term should
     * be calculated. If the absolute value of the raw derivative exceeds the
     * stability threshold, the integral term will not be updated. This can help
     * prevent integral windup and improve stability in scenarios where the system
     * may experience rapid changes or noise that could cause large derivative
     * values.
     *
     * @param threshold The stability threshold value.
     * @return An Option that sets the stability threshold of the PID controller to
     *         the specified value.
     */

    public static Option withStabilityThreshold(double threshold) {
        return p -> p.stabilityThreshold = Math.abs(threshold);
    }

    /**
     * Creates an Option that sets the maximum absolute value for the integral sum.
     * If the integral sum exceeds this value, it will be clamped to the maximum.
     * This can help prevent integral windup and improve stability in scenarios
     * where the system may experience sustained errors that could cause the
     * integral term to grow excessively large.
     *
     * @param maxSum The maximum absolute value for the integral sum.
     * @return An Option that sets the integralSumMax of the PID controller to the
     *         specified value.
     */

    public static Option withIntegralSumMax(double maxSum) {
        return p -> p.integralSumMax = Math.abs(maxSum);
    }

    /**
     * Creates an Option that sets the filter to be used for smoothing the
     * derivative term. The filter will be applied to the raw derivative value
     * before it is multiplied by the derivative gain (kd). This can help reduce
     * noise and improve stability in scenarios where the system may experience
     * rapid changes or noise that could cause large derivative values.
     *
     * @param filter The filter to be used for smoothing the derivative term.
     * @return An Option that sets the filter of the PID controller to the specified
     *         filter instance.
     */

    public static Option withFilter(Filter filter) {
        return p -> p.filter = filter;
    }

    /**
     * Creates an Option that sets the output limits for the PID controller. The
     * output of the PID controller will be clamped to the specified minimum and
     * maximum values. This can help prevent the output from exceeding the physical
     * limits of the system or causing instability due to excessively large outputs.
     *
     * @param min The minimum output value for the PID controller.
     * @param max The maximum output value for the PID controller.
     * @return An Option that sets the output limits of the PID controller to the
     *         specified minimum and maximum values.
     */

    public static Option withOutputLimits(double min, double max) {
        return p -> {
            if (min <= max) {
                p.outputMin = min;
                p.outputMax = max;
            }
        };
    }

    /**
     * Creates an Option that sets the PID gains based on a damped
     * spring-mass-damper system model. The gains are calculated based on the
     * specified spring stiffness (ka), mass (kv), and percent overshoot (po). This
     * can help simplify the tuning process by providing a systematic way to
     * calculate the PID gains based on the desired dynamic response of the system.
     *
     * @param ka The spring stiffness, which represents the proportional gain (kp)
     *           of the system. A higher ka will result in a stronger proportional
     *           response to errors, which can help reduce steady-state error and
     *           improve responsiveness. However, if ka is too high, it can lead to
     *           overshooting and oscillations.
     * @param kv The mass, which represents the derivative gain (kd) of the system.
     *           A higher kv will result in a stronger derivative response to
     *           changes in error, which can help dampen oscillations and improve
     *           stability. However, if kv is too high, it can lead to excessive
     *           damping and slow response.
     * @param po The percent overshoot, which is used to calculate the damping ratio
     *           (zeta) of the system. A higher po will result in a less damped
     *           system with more overshoot, while a lower po will result in a more
     *           damped system with less overshoot. The po value should be between 0
     *           and 100, where 0 represents a critically damped system with no
     *           overshoot, and 100 represents an undamped system with maximum
     *           overshoot. If po is set to 0, the derivative gain (kd) will be
     *           calculated for critical damping, which can help achieve a fast
     *           response without overshooting. If po is greater than 0, the
     *           derivative gain will be calculated based on the specified percent
     *           overshoot, which can help achieve a desired level of damping and
     *           overshoot in the system's response.
     * @return An Option that sets the PID gains of the controller based on the
     *         specified spring stiffness, mass,
     */

    public static Option withDampening(double ka, double kv, double po) {
        return p -> {
            if (p.kp < kv * kv / 4 * ka) {
                return;
            }

            if (po == 0) {
                p.kd = 2 * Math.sqrt(ka * kv) - ka;
            } else {
                double boundedPo = Math.max(po / 100.0, 0.01);
                double poLog = Math.log(boundedPo);
                double zeta = -poLog / Math.sqrt(Math.PI * Math.PI + poLog * poLog);
                p.kd = 2 * zeta * Math.sqrt(ka * kv) - kv;
            }
        };
    }

    /**
     * Calculates the PID output based on the given reference and current state. The
     * method computes the proportional, integral, and derivative terms based on the
     * error between the reference and state, and applies any configured
     * feedforward, stability threshold, integral sum limits, and output limits.
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
        double error = reference - state;

        if (!initialized) {
            integral = 0;
            lastReference = reference;
            lastError = error;
            prevTimeNanos = now;
            initialized = true;
        }

        if (reference != lastReference) {
            integral = 0;
            lastReference = reference;
        }

        double dt = (now - prevTimeNanos) / 1_000_000_000.0;

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
        prevTimeNanos = now;

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
