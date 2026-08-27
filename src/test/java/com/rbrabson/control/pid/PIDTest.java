package com.rbrabson.control.pid;

import org.junit.jupiter.api.Test;

import com.rbrabson.control.filter.Filter;
import com.rbrabson.control.filter.LowPassFilter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PIDTest {
    @Test
    void respectsOutputLimits() {
        PID pid = new PID(10.0, 0.0, 0.0).withOutputLimits(-5.0, 5.0);
        double out = pid.calculate(10.0, 0.0);
        assertTrue(out <= 5.0 && out >= -5.0);
    }

    @Test
    void supportsDerivativeFilterOption() {
        PID pid = new PID(0.0, 0.0, 1.0).withFilter(new LowPassFilter(0.8));
        pid.calculate(10.0, 0.0);
        double out = pid.calculate(8.0, 0.0);
        assertTrue(Double.isFinite(out));
    }

    @Test
    void evaluatesDerivativeFilterOncePerUpdate() {
        CountingFilter filter = new CountingFilter();
        PID pid = new PID(0.0, 0.0, 1.0).withFilter(filter);

        pid.calculate(0.0, 0.0, 0.0);
        pid.calculate(10.0, 0.0, 1.0);

        assertEquals(1, filter.calls);
        assertEquals(10.0, filter.lastMeasurement);
    }

    @Test
    void stabilityThresholdUsesUnfilteredDerivative() {
        Filter filter = new Filter() {
            @Override
            public double estimate(double measurement) {
                return 0.0;
            }

            @Override
            public void reset() {
            }
        };
        PID pid = new PID(0.0, 1.0, 0.0)
                .withFilter(filter)
                .withStabilityThreshold(1.0);

        pid.calculate(0.0, 0.0, 0.0);
        double out = pid.calculate(10.0, 0.0, 1.0);

        assertEquals(0.0, out, 0.0);
    }

    @Test
    void antiWindupHonorsIntegralSumMaximum() {
        PID pid = new PID(0.0, 1.0, 0.0)
                .withFeedForward(100.0)
                .withIntegralSumMax(1.0)
                .withOutputLimits(-1.0, 1.0);

        pid.calculate(1.0, 0.0, 1.0);
        pid.withOutputLimits(-1000.0, 1000.0);

        assertEquals(99.0, pid.calculate(1.0, 0.0, 0.0), 0.0);
    }

    @Test
    void rejectsInvalidTimeStep() {
        PID pid = new PID(1.0);

        assertThrows(IllegalArgumentException.class, () -> pid.calculate(0.0, 0.0, -1.0));
        assertThrows(IllegalArgumentException.class, () -> pid.calculate(0.0, 0.0, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> pid.calculate(0.0, 0.0, Double.POSITIVE_INFINITY));
    }

    @Test
    void rejectsInvalidOutputLimits() {
        PID pid = new PID(1.0);

        assertThrows(IllegalArgumentException.class, () -> pid.withOutputLimits(1.0, -1.0));
        assertThrows(IllegalArgumentException.class, () -> pid.withOutputLimits(Double.NaN, 1.0));
        assertThrows(IllegalArgumentException.class, () -> pid.withOutputLimits(0.0, Double.POSITIVE_INFINITY));
    }

    @Test
    void rejectsNonFiniteConfigurationAndInputs() {
        assertThrows(IllegalArgumentException.class, () -> new PID(Double.NaN));
        PID pid = new PID(1.0);
        assertThrows(IllegalArgumentException.class, () -> pid.withFeedForward(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> pid.withStabilityThreshold(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> pid.withIntegralSumMax(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> pid.calculate(Double.NaN, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> pid.calculate(0.0, Double.POSITIVE_INFINITY, 0.0));
    }

    @Test
    void validatesDampeningParameters() {
        PID pid = new PID(1.0);

        assertThrows(IllegalArgumentException.class, () -> pid.withDampening(0.0, 1.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> pid.withDampening(1.0, 1.0, -1.0));
        assertThrows(IllegalArgumentException.class, () -> pid.withDampening(Double.NaN, 1.0, 0.0));
    }

    @Test
    void calculatesStandardCriticalDampingGain() {
        PID pid = new PID(1.0).withDampening(4.0, 9.0, 0.0);

        pid.calculate(0.0, 0.0, 0.0);
        assertEquals(-13.0, pid.calculate(0.0, 1.0, 1.0), 1e-9);
    }

    @Test
    void fluentConfigurationMutatesAndReturnsThisController() {
        PID pid = new PID(1.0);

        assertTrue(pid == pid.withFeedForward(2.0).withOutputLimits(-1.0, 1.0));
        assertEquals(1.0, pid.calculate(10.0, 0.0, 0.0), 0.0);
    }

    private static final class CountingFilter implements Filter {
        private int calls;
        private double lastMeasurement;

        @Override
        public double estimate(double measurement) {
            calls++;
            lastMeasurement = measurement;
            return measurement;
        }

        @Override
        public void reset() {
        }
    }
}
