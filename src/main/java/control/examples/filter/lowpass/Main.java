package control.examples.filter.lowpass;

import control.filter.LowPassFilter;

public class Main {
    public static void main(String[] args) {
        LowPassFilter filter = new LowPassFilter(0.6);
        double[] signal = { 0, 10, 4, 12, 5, 11 };

        for (double measurement : signal) {
            double estimate = filter.estimate(measurement);
            System.out.printf("Raw=%.2f Filtered=%.2f%n", measurement, estimate);
        }
    }
}
