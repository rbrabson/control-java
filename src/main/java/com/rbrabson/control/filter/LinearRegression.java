package com.rbrabson.control.filter;

import java.util.Arrays;

/**
 * A simple linear regression implementation that fits a line to a set of data
 * points and predicts the next value in the sequence. The class uses the least
 * squares method to calculate the slope and intercept of the best fit line.
 */
public class LinearRegression {
    private double[] data;
    private double slope;
    private double intercept;
    private boolean hasRun;

    /**
     * Initializes the linear regression with the given data. The data is copied to
     * ensure that the original array can be modified without affecting the
     * regression calculations.
     *
     * @param data The input data for the linear regression, where each element
     *             represents a y-value and the x-values are implicitly the indices
     *             of the array.
     */
    public LinearRegression(double[] data) {
        validateData(data);
        this.data = Arrays.copyOf(data, data.length);
    }

    /**
     * Runs the least squares method to calculate the slope and intercept of the
     * best fit line for the given data. The method handles edge cases such as when
     * there are fewer than two data points, in which case it defaults to a slope of
     * 0 and an intercept equal to the average of the data points (or 0 if there are
     * no data points). the calculations have been performed.
     */
    public void runLeastSquares() {
        int n = data.length;
        if (n < 2) {
            slope = 0;
            intercept = n == 0 ? 0 : data[0];
            hasRun = true;
            return;
        }

        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumXX = 0.0;

        for (int i = 0; i < data.length; i++) {
            double x = i;
            double y = data[i];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
            if (!Double.isFinite(sumX) || !Double.isFinite(sumY)
                    || !Double.isFinite(sumXY) || !Double.isFinite(sumXX)) {
                throw new IllegalArgumentException("regression calculation overflowed");
            }
        }

        double nf = n;
        double denominator = nf * sumXX - sumX * sumX;
        if (!Double.isFinite(denominator)) {
            throw new IllegalArgumentException("regression calculation overflowed");
        }
        if (Math.abs(denominator) < 1e-10) {
            slope = 0;
            intercept = n > 0 ? sumY / nf : 0;
        } else {
            slope = (nf * sumXY - sumX * sumY) / denominator;
            intercept = (sumY - slope * sumX) / nf;
            if (!Double.isFinite(slope) || !Double.isFinite(intercept)) {
                throw new IllegalArgumentException("regression calculation overflowed");
            }
        }

        hasRun = true;
    }

    /**
     * Predicts the next value in the sequence based on the calculated slope and
     * intercept. If the least squares method has not been run yet, it will run it
     * before making the prediction. If there is only one data point, it will return
     * that point as the prediction for the next value, since we cannot determine a
     * slope from a single point.
     *
     * @return The predicted next value in the sequence based on the linear
     *         regression model.
     */
    public double predictNextValue() {
        if (!hasRun) {
            runLeastSquares();
        }

        if (data.length == 1) {
            return data[0];
        }

        double nextX = data.length;
        return slope * nextX + intercept;
    }

    /**
     * Updates the data for the linear regression and resets the hasRun flag to
     * indicate that the least squares method needs to be run again before making
     * predictions. The new data is copied to ensure that modifications to the
     * original array do not affect the regression calculations.
     *
     * @param data The new input data for the linear regression, where each element
     *             represents a y-value and the x-values are implicitly the indices
     *             of the array.
     */
    public void updateData(double[] data) {
        validateData(data);
        this.data = Arrays.copyOf(data, data.length);
        this.hasRun = false;
    }

    private static void validateData(double[] data) {
        if (data == null) {
            throw new IllegalArgumentException("data must be non-null");
        }
        for (double value : data) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException("data values must be finite");
            }
        }
    }
}
