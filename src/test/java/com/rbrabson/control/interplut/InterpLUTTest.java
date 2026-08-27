package com.rbrabson.control.interplut;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

class InterpLUTTest {
    @Test
    void returnsExactControlPoints() {
        InterpLUT lut = new InterpLUT().withPoint(0.0, 0.0).withPoint(1.0, 1.0).build();

        assertEquals(0.0, lut.get(0.0), 1e-9);
        assertEquals(1.0, lut.get(1.0), 1e-9);
    }

    @Test
    void sortsByXAndInterpolatesUnsortedPoints() {
        InterpLUT lut = new InterpLUT()
                .withPoint(2.0, 2.0)
                .withPoint(0.0, 0.0)
                .withPoint(1.0, 1.0);

        assertEquals(0.5, lut.get(0.5), 1e-9);
    }

    @Test
    void addingPointAfterBuildRebuildsSlopes() {
        InterpLUT lut = new InterpLUT().withPoint(0.0, 0.0).withPoint(1.0, 1.0).build();
        InterpLUT extended = lut.withPoint(2.0, 4.0);

        assertEquals(2.375, extended.get(1.5), 1e-9);
    }

    @Test
    void rejectsInvalidControlPointsAndDuplicateXValues() {
        assertThrows(IllegalArgumentException.class, () -> new InterpLUT().withPoint(Double.NaN, 0.0));
        assertThrows(IllegalStateException.class, () -> new InterpLUT()
                .withPoint(0.0, 0.0).withPoint(0.0, 1.0).build());
    }

    @Test
    void toStringIsSafeBeforeBuild() {
        InterpLUT lut = new InterpLUT().withPoint(0.0, 0.0).withPoint(1.0, 1.0);

        assertDoesNotThrow(() -> lut.toString());
    }
}
