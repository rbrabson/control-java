package com.rbrabson.control.feedback;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class FullStateFeedbackTest {
    @Test
    void calculatesDotProductOfErrorAndGain() {
        FullStateFeedback fsf = new FullStateFeedback(new double[] { 1.5, 0.3 });
        double out = fsf.calculate(new double[] { 10.0, 0.0 }, new double[] { 8.0, 1.0 });
        assertEquals(2.7, out, 1e-9);
    }

    @Test
    void throwsOnMismatchedVectorLengths() {
        FullStateFeedback fsf = new FullStateFeedback(new double[] { 1.0, 1.0 });
        assertThrows(IllegalArgumentException.class, new org.junit.jupiter.api.function.Executable() {
            @Override
            public void execute() throws Throwable {
                fsf.calculate(new double[] { 1.0 }, new double[] { 1.0, 2.0 });
            }
        });
    }
}
