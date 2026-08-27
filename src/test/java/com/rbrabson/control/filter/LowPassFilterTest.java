package com.rbrabson.control.filter;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class LowPassFilterTest {
    @Test
    void appliesFirstOrderLowPassFilter() {
        LowPassFilter lpf = new LowPassFilter(0.5);
        assertEquals(10.0, lpf.estimate(10.0), 1e-9);
        assertEquals(15.0, lpf.estimate(20.0), 1e-9);
    }

    @Test
    void rejectsNonFiniteAlpha() {
        assertThrows(IllegalArgumentException.class, () -> new LowPassFilter(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new LowPassFilter(Double.POSITIVE_INFINITY));

        LowPassFilter lpf = new LowPassFilter(0.5);
        assertThrows(IllegalArgumentException.class, () -> lpf.setAlpha(Double.NaN));
    }
}
