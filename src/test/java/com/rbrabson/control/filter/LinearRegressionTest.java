package com.rbrabson.control.filter;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class LinearRegressionTest {
    @Test
    void predictsSinglePointAsThatPoint() {
        LinearRegression regression = new LinearRegression(new double[] { 7.0 });

        assertEquals(7.0, regression.predictNextValue(), 1e-9);
    }

    @Test
    void predictsNextValueForLinearData() {
        LinearRegression regression = new LinearRegression(new double[] { 1.0, 3.0, 5.0 });

        assertEquals(7.0, regression.predictNextValue(), 1e-9);
    }

    @Test
    void rejectsInvalidData() {
        assertThrows(IllegalArgumentException.class, () -> new LinearRegression(null));
        assertThrows(IllegalArgumentException.class,
                () -> new LinearRegression(new double[] { 1.0, Double.NaN }));
    }
}
