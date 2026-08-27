package com.rbrabson.control.filter;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class KalmanFilterTest {
    @Test
    void validatesConfigurationAndState() {
        assertThrows(IllegalArgumentException.class, () -> new KalmanFilter(Double.NaN, 1.0, 3));
        assertThrows(IllegalArgumentException.class, () -> new KalmanFilter(1.0, Double.POSITIVE_INFINITY, 3));

        KalmanFilter filter = new KalmanFilter(0.1, 0.1, 3);
        assertThrows(IllegalArgumentException.class, () -> filter.setX(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> filter.estimate(Double.NaN));
    }

    @Test
    void estimatesFiniteMeasurements() {
        KalmanFilter filter = new KalmanFilter(0.1, 0.1, 3);

        assertTrue(Double.isFinite(filter.estimate(10.0)));
        assertTrue(Double.isFinite(filter.estimate(10.0)));
    }
}
