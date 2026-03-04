package control.pid;

import control.filter.Filter;
import java.util.function.Consumer;
import java.util.logging.Logger;

public class PID {
    public interface Option extends Consumer<PID> {
    }

    private static final Logger LOGGER = Logger.getLogger(PID.class.getName());

    private double kp;
    private double ki;
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

    public static Option withFeedForward(double feedForward) {
        return p -> p.feedForward = feedForward;
    }

    public static Option withIntegralResetOnZeroCross() {
        return p -> p.integralResetOnZeroCross = true;
    }

    public static Option withStabilityThreshold(double threshold) {
        return p -> p.stabilityThreshold = Math.abs(threshold);
    }

    public static Option withIntegralSumMax(double maxSum) {
        return p -> p.integralSumMax = Math.abs(maxSum);
    }

    public static Option withFilter(Filter filter) {
        return p -> p.filter = filter;
    }

    public static Option withOutputLimits(double min, double max) {
        return p -> {
            if (min <= max) {
                p.outputMin = min;
                p.outputMax = max;
            }
        };
    }

    public static Option withDampening(double ka, double kv, double po) {
        return p -> {
            if (p.kp < kv * kv / 4 * ka) {
                LOGGER.severe("invalid kp, kv, and ka values for PID.withDampening");
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
        double clampedOutput = clamp(output);

        if (output != clampedOutput && ki != 0) {
            integral = (clampedOutput - proportional - derivative - feedForward) / ki;
        }

        lastError = error;
        prevTimeNanos = now;

        return clampedOutput;
    }

    private double calculateProportional(double error) {
        return kp * error;
    }

    private double calculateIntegral(double error, double dt) {
        if (integralResetOnZeroCross && ((lastError > 0 && error < 0) || (lastError < 0 && error > 0))) {
            integral = 0;
        }

        double rawDerivative = calculateRawDerivative(error, dt);
        if (Double.isNaN(stabilityThreshold) || Math.abs(rawDerivative) <= stabilityThreshold) {
            integral += error * dt;

            if (!Double.isNaN(integralSumMax)) {
                if (integral > integralSumMax) {
                    integral = integralSumMax;
                } else if (integral < -integralSumMax) {
                    integral = -integralSumMax;
                }
            }
        }

        return ki * integral;
    }

    private double calculateDerivative(double error, double dt) {
        double rawDerivative = calculateRawDerivative(error, dt);
        return kd * rawDerivative;
    }

    private double calculateRawDerivative(double error, double dt) {
        if (dt <= 0) {
            return 0;
        }

        double errorChange = error - lastError;
        double currentEstimate = filter != null ? filter.estimate(errorChange) : errorChange;
        return currentEstimate / dt;
    }

    private double clamp(double value) {
        if (value > outputMax) {
            return outputMax;
        }
        if (value < outputMin) {
            return outputMin;
        }
        return value;
    }

    public PID reset() {
        integral = 0;
        initialized = false;
        lastError = 0;
        if (filter != null) {
            filter.reset();
        }
        return this;
    }

    public PID setGains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        return this;
    }

    public double[] getGains() {
        return new double[] { kp, ki, kd };
    }

    public double getIntegral() {
        return integral;
    }

    public PID setFeedForward(double feedForward) {
        this.feedForward = feedForward;
        return this;
    }

    public double getFeedForward() {
        return feedForward;
    }

    public PID setIntegralResetOnZeroCross(boolean enabled) {
        integralResetOnZeroCross = enabled;
        return this;
    }

    public boolean getIntegralResetOnZeroCross() {
        return integralResetOnZeroCross;
    }

    public PID setStabilityThreshold(double threshold) {
        stabilityThreshold = Math.abs(threshold);
        return this;
    }

    public double getStabilityThreshold() {
        return stabilityThreshold;
    }

    public PID setIntegralSumMax(double maxSum) {
        integralSumMax = Math.abs(maxSum);
        return this;
    }

    public double getIntegralSumMax() {
        return integralSumMax;
    }

    public PID setFilter(Filter filter) {
        this.filter = filter;
        return this;
    }

    public Filter getFilter() {
        return filter;
    }

    public PID setOutputLimits(double min, double max) {
        if (min > max) {
            return this;
        }

        outputMin = min;
        outputMax = max;
        integral = clamp(integral);
        return this;
    }

    public double[] getOutputLimits() {
        return new double[] { outputMin, outputMax };
    }
}
