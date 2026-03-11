package com.rbrabson.control.examples.pid.position_servo;

import com.rbrabson.control.pid.PID;

public class Main {
    public static void main(String[] args) {
        // Servo position control with proportional-only control
        // For direct velocity control, P-only is often sufficient
        PID controller = new PID(1.0, 0.0, 0.0).withOutputLimits(-1.0, 1.0);

        double targetDeg = 90.0;
        double position = 0.0;
        double velocity = 0.0;
        double dt = 0.01; // 10ms

        System.out.println("Servo Position Control");
        System.out.println("======================");
        System.out.println("Moving servo from 0° to 90°\n");
        System.out.printf("%-8s %-12s %-12s %-12s %-12s%n", "Time", "Position", "Velocity", "Error", "Command");
        System.out.println("----------------------------------------------------------");

        for (int i = 0; i <= 300; i++) {
            double time = i * dt;
            double command = controller.calculate(targetDeg, position, dt);

            // Servo dynamics: command directly controls velocity
            velocity = command * 100.0; // Command scales directly to velocity (degrees/s)
            position += velocity * dt;

            if (i % 30 == 0) {
                double error = targetDeg - position;
                System.out.printf("%.2f     %8.2f°    %8.2f     %8.2f°    %8.3f%n", time, position, velocity, error,
                        command);
            }
        }

        System.out.println(
                "\nNote: Servo velocity is directly proportional to PID command (velocity = command × 100°/s)");
        System.out.println("      Pure proportional controller (P-only) is sufficient for direct velocity control.");
        System.out.println("      Uses PID calculate(ref, state, dt) for simulation without real-time dependency.");
    }
}
