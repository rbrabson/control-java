package com.rbrabson.control.interplut;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class InterpLUTTest {
    @Test
    void returnsExactControlPoints() {
        InterpLUT lut = new InterpLUT().withPoint(0.0, 0.0).withPoint(1.0, 1.0).build();

        assertEquals(0.0, lut.get(0.0), 1e-9);
        assertEquals(1.0, lut.get(1.0), 1e-9);
    }
}
