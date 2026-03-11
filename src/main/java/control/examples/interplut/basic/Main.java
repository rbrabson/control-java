package control.examples.interplut.basic;

import control.interplut.InterpLUT;

public class Main {
    public static void main(String[] args) {
        // Interpolated lookup table maps RPM to throttle percentage
        // Common use: non-linear mappings, calibration tables, etc.
        InterpLUT throttleMap = new InterpLUT().withPoint(0, 0.0).withPoint(1000, 0.2).withPoint(2000, 0.55)
                .withPoint(3000, 1.0).build();

        System.out.println("Interpolated Lookup Table (LUT)");
        System.out.println("================================");
        System.out.println("Throttle mapping: RPM → Throttle %\n");
        System.out.printf("%-12s %-15s %-20s%n", "RPM", "Throttle %", "Note");
        System.out.println("---------------------------------------------------");

        // Test various RPM values including interpolated ones
        double[] testRpms = { 0, 500, 1000, 1500, 2000, 2500, 3000 };

        for (double rpm : testRpms) {
            double throttle = throttleMap.get(rpm);
            String note = "";
            if (rpm == 0 || rpm == 1000 || rpm == 2000 || rpm == 3000) {
                note = "(exact point)";
            } else {
                note = "(interpolated)";
            }
            System.out.printf("%8.0f     %10.3f      %s%n", rpm, throttle, note);
        }

        System.out.println("\nNote: Values between points are linearly interpolated");

        // Demonstrate bounds checking
        System.out.println("\nBounds checking demonstration:");
        try {
            double outOfBounds = throttleMap.get(3500);
            System.out.printf("3500 RPM: %.3f (unexpected)\n", outOfBounds);
        } catch (IllegalArgumentException e) {
            System.out.println("3500 RPM: Out of bounds (0-3000) - exception thrown ✓");
        }
    }
}
