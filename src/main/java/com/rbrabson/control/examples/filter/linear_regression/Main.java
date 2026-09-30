package com.rbrabson.control.examples.filter.linear_regression;

import com.rbrabson.control.filter.LinearRegression;
import com.rbrabson.control.filter.SizedStack;

public class Main {
    public static void main(String[] args) {
        // Maintain a rolling window of recent measurements, then fit a line to estimate
        // the next sample. This pattern is useful for trend detection and prediction.
        SizedStack<Double> history = new SizedStack<>(6);
        double[] noisyTrend = { 10.1, 11.2, 12.1, 13.0, 13.8, 15.2, 15.9, 17.0, 18.1, 19.0 };

        System.out.println("Linear Regression Trend Forecast");
        System.out.println("==============================");
        System.out.println("Using a rolling window of recent values to estimate the next trend point\n");
        System.out.printf("%-8s %-12s %-18s %-12s %-12s%n", "Step", "Measured", "Rolling Window", "PredictNext", "Error");
        System.out.println("--------------------------------------------------------------------------");

        for (int i = 0; i < noisyTrend.length; i++) {
            history.push(noisyTrend[i]);

            double[] window = new double[history.size()];
            for (int j = 0; j < history.size(); j++) {
                window[j] = history.get(j);
            }

            LinearRegression regression = new LinearRegression(window);
            double predicted = regression.predictNextValue();
            double trueTrend = 10.0 + (i + 1);
            double error = Math.abs(trueTrend - predicted);

            System.out.printf("%4d     %8.2f     %s     %8.3f     %8.3f%n",
                    i + 1,
                    noisyTrend[i],
                    formatWindow(window),
                    predicted,
                    error);
        }

        System.out.println("\nObservations:");
        System.out.println("- SizedStack keeps a fixed-length rolling window of recent measurements.");
        System.out.println("- LinearRegression fits a straight line to that history and predicts the next value.");
        System.out.println("- This works well when the underlying signal is approximately linear and noisy.");
    }

    private static String formatWindow(double[] values) {
        StringBuilder builder = new StringBuilder("[");
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                builder.append(", ");
            }
            builder.append(String.format("%.1f", values[i]));
        }
        builder.append("]");
        return builder.toString();
    }
}
