package com.rbrabson.control.examples.filter.basic;

import com.rbrabson.control.filter.KalmanFilter;

public class Main {
    public static void main(String[] args) {
        // Kalman filter with regression-based prediction for trending data
        // Larger stack size (8) allows better trend detection
        KalmanFilter kf = new KalmanFilter(0.05, 0.3, 8);
        kf.setX(10.0); // Set initial estimate

        System.out.println("Kalman Filter with Regression Example");
        System.out.println("======================================");
        System.out.println("Tracking a linearly increasing value with noisy measurements\n");
        System.out.printf("%-8s %-12s %-12s %-12s %-12s %-12s%n", "Step", "True", "Measurement", "Estimate", "Error",
                "Gain");
        System.out.println("-------------------------------------------------------------------------");

        // True value increases linearly: 10, 11, 12, 13...
        // Measurements have noise of ±0.3
        double[] measurements = { 10.1, 10.9, 12.2, 13.1, 13.8, 15.2, 15.9, 17.1, 17.8, 19.2, 19.9, 21.1, 21.8, 23.2,
                23.9 };

        for (int i = 0; i < measurements.length; i++) {
            double trueValue = 10.0 + i;
            double measurement = measurements[i];
            double estimate = kf.estimate(measurement);
            double error = Math.abs(trueValue - estimate);

            System.out.printf("%4d     %8.1f     %8.3f     %8.3f     %8.3f     %8.4f%n", i + 1, trueValue, measurement,
                    estimate, error, kf.getK());
        }

        System.out.println("\nNote: This filter uses linear regression to predict the next value based on trend");
        System.out.println("      With larger stack size (n=8), regression better captures the linear trend");
        System.out.println("      Kalman gain balances between trend prediction and noisy measurements");
    }
}
