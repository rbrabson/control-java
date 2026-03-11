package control.examples.filter.lowpass;

import control.filter.LowPassFilter;

public class Main {
    public static void main(String[] args) {
        // Low-pass filter comparison using standard alpha terminology
        // Alpha = weight on new measurement (standard convention)
        // Higher alpha = less smoothing (responds faster to changes)
        // Lower alpha = more smoothing (removes more noise but slower response)
        LowPassFilter filterHigh = new LowPassFilter(0.8); // High alpha = fast response
        LowPassFilter filterMid = new LowPassFilter(0.5); // Medium alpha = balanced
        LowPassFilter filterLow = new LowPassFilter(0.2); // Low alpha = heavy smoothing

        System.out.println("Low-Pass Filter Comparison");
        System.out.println("===========================");
        System.out.println("Filtering noisy signal with different smoothing factors\n");
        System.out.printf("%-8s %-10s %-15s %-15s %-15s%n", "Step", "Raw", "Alpha=0.8", "Alpha=0.5", "Alpha=0.2");
        System.out.println("----------------------------------------------------------------");

        // Simulated noisy signal that steps from 0 to 10
        double[] signal = { 0.0, 0.5, -0.3, 0.2, 10.2, 9.8, 10.5, 9.6, 10.3, 10.1, 9.9, 10.4, 10.0, 9.7, 10.2, 10.1 };

        for (int i = 0; i < signal.length; i++) {
            double raw = signal[i];
            double estHigh = filterHigh.estimate(raw);
            double estMid = filterMid.estimate(raw);
            double estLow = filterLow.estimate(raw);

            System.out.printf("%4d     %6.2f     %11.3f     %11.3f     %11.3f%n", i + 1, raw, estHigh, estMid, estLow);
        }

        System.out.println("\nObservations:");
        System.out.println("- Alpha=0.8 (high): Faster response, reaches target quickly but retains more noise");
        System.out.println("- Alpha=0.5 (mid): Balanced trade-off between speed and smoothing");
        System.out.println("- Alpha=0.2 (low): Smoothest output but slowest to respond to step change");
        System.out.println("\nNote: Alpha represents weight on new measurement (standard convention)");
    }
}
