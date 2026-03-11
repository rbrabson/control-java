package control.examples.interplut.temperature;

import control.interplut.InterpLUT;

public class Main {
    public static void main(String[] args) {
        // Temperature compensation table for sensor calibration
        // Real sensors often need compensation for ambient temperature effects
        InterpLUT compensation = new InterpLUT().withPoint(0, 1.0) // 0°C: no compensation needed
                .withPoint(20, 0.95) // Room temp: slight correction
                .withPoint(40, 0.90) // Warm: more correction
                .withPoint(60, 0.82) // Hot: significant correction
                .build();

        System.out.println("Temperature Compensation Table");
        System.out.println("==============================");
        System.out.println("Compensates sensor gain for ambient temperature\n");
        System.out.printf("%-15s %-15s %-20s%n", "Ambient(°C)", "Scale Factor", "Correction(%)");
        System.out.println("------------------------------------------------");

        double[] temperatures = { 0, 10, 20, 30, 40, 50, 60 };

        for (double temp : temperatures) {
            double gainScale = compensation.get(temp);
            double correction = (1.0 - gainScale) * 100;
            System.out.printf("%10.1f      %10.4f      %10.1f%%%n", temp, gainScale, correction);
        }

        System.out.println("\nExample usage:");
        double ambient = 35.0;
        double rawReading = 100.0;
        double scaleFactor = compensation.get(ambient);
        double compensated = rawReading * scaleFactor;
        System.out.printf("At %.1f°C: raw=%.1f → compensated=%.2f (scale=%.4f)%n", ambient, rawReading, compensated,
                scaleFactor);

        System.out.println("\nBounds checking:");
        try {
            double outOfBounds = compensation.get(70);
            System.out.printf("70°C: %.4f (unexpected)\n", outOfBounds);
        } catch (IllegalArgumentException e) {
            System.out.println("70°C: Out of bounds (0-60°C) - exception thrown ✓");
        }
    }
}
