package control.feedforward;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class FeedForwardTest {
    @Test
    void includesGravityAndCosineTerms() {
        FeedForward ff = new FeedForward(0.0, 2.0, 3.0).withGravityGain(5.0).withCosineGain(2.0);

        double out = ff.calculate(Math.PI, 1.0, 2.0);
        assertEquals(11.0, out, 1e-9);
    }
}
