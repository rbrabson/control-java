package com.rbrabson.control.examples.pid.basic_control_loop;

import com.rbrabson.control.pid.PID;

public class Main {
    public static void main(String[] args) {
        // Basic PID controller: Kp=4.0, Ki=1.2, Kd=3.0
        // Tuned for fast response with minimal overshoot
        PID controller = new PID(4.0, 1.2, 3.0).withOutputLimits(-100, 100);

        double setpoint = 50.0;
        double position = 0.0;
        double velocity = 0.0;
        double dt = 0.02; // 20ms time step

        System.out.println("Basic PID Control Loop");
        System.out.println("======================");
        System.out.println("Target: 50.0, Starting: 0.0");
        System.out.println("Gains: Kp=4.0, Ki=1.2, Kd=3.0\n");
        System.out.printf("%-8s %-12s %-12s %-12s%n", "Time", "Position", "Error", "Output");
        System.out.println("-----------------------------------------------");

        for (int i = 0; i <= 400; i++) {
            double time = i * dt;
            double output = controller.calculate(setpoint, position, dt);
            velocity += output * dt;
            position += velocity * dt;

            if (i % 20 == 0) {
                double error = setpoint - position;
                System.out.printf("%.2f     %8.3f     %8.3f     %8.3f%n", time, position, error, output);
            }
        }

        System.out.println("\nFinal position: " + String.format("%.3f", position) + " (target: 50.0)");
        System.out.println("\nHow PID works:");
        System.out.println("- Proportional (P): Drives toward target based on current error");
        System.out.println("- Integral (I): Eliminates steady-state error by accumulating past errors");
        System.out.println("- Derivative (D): Provides damping to prevent overshoot");
    }
}
