package com.rbrabson.control.feedforward;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class FeedForwardTest {
    @Test
    void includesGravityAndCosineTerms() {
        FeedForward ff = new FeedForward(5.0, 2.0, 3.0).withCosineGain(2.0);

        double out = ff.calculate(Math.PI, 1.0, 2.0);
        assertEquals(11.0, out, 1e-9);
    }

    @Test
    void rejectsNonFiniteGains() {
        assertThrows(IllegalArgumentException.class, () -> new FeedForward(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new FeedForward(0.0, Double.POSITIVE_INFINITY));

        FeedForward ff = new FeedForward(0.0);
        assertThrows(IllegalArgumentException.class, () -> ff.withCosineGain(Double.NaN));
    }

    @Test
    void appliesStaticGainInDirectionOfMotion() {
        FeedForward ff = new FeedForward(2.0, 1.0, 1.0);

        assertEquals(3.0, ff.calculate(0.0, 1.0, 0.0), 1e-9);
        assertEquals(-3.0, ff.calculate(0.0, -1.0, 0.0), 1e-9);
        assertEquals(0.0, new FeedForward(2.0).calculate(0.0, 0.0, 0.0), 1e-9);
    }
}
