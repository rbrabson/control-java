package control.examples.filter.basic;

import control.filter.KalmanFilter;

public class Main {
    public static void main(String[] args) {
        KalmanFilter kf = new KalmanFilter(0.05, 0.1, 10);

        double[] measurements = { 10.0, 11.2, 9.8, 10.6, 10.3 };
        for (double measurement : measurements) {
            double estimate = kf.estimate(measurement);
            System.out.printf("Measurement=%.2f Estimate=%.2f Gain=%.4f%n", measurement, estimate, kf.getGain());
        }
    }
}
