package com.rbrabson.control.filter;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class LowPassFilterTest {
    @Test
    void appliesFirstOrderLowPassFilter() {
        LowPassFilter lpf = new LowPassFilter(0.5);
        assertEquals(10.0, lpf.estimate(10.0), 1e-9);
        assertEquals(15.0, lpf.estimate(20.0), 1e-9);
    }
}
