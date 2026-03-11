package com.rbrabson.control.examples.motionprofile.basic;

import com.rbrabson.control.motionprofile.Constraints;
import com.rbrabson.control.motionprofile.MotionProfile;
import com.rbrabson.control.motionprofile.State;

public class Main {
    public static void main(String[] args) {
        // Generate a trapezoidal motion profile
        // Constraints: maxVelocity=2.0, maxAcceleration=1.0
        MotionProfile profile = new MotionProfile(new Constraints(2.0, 1.0), new State(0.0, 0.0, 0.0, 0.0), // Start:
                                                                                                            // pos=0,
                                                                                                            // vel=0
                new State(5.0, 0.0, 0.0, 0.0) // Goal: pos=5, vel=0
        );

        System.out.println("Trapezoidal Motion Profile");
        System.out.println("===========================");
        System.out.printf("Move from 0 to 5 units in %.3f seconds%n", profile.totalTime());
        System.out.println("Max velocity: 2.0 m/s, Max acceleration: 1.0 m/s²\n");
        System.out.printf("%-8s %-12s %-12s %-12s %-15s%n", "Time", "Position", "Velocity", "Accel", "Phase");
        System.out.println("------------------------------------------------------------");

        for (double t = 0.0; t <= profile.totalTime(); t += 0.5) {
            State s = profile.calculate(t);
            String phase = "";
            if (s.acceleration > 0.01) {
                phase = "Accelerating";
            } else if (s.acceleration < -0.01) {
                phase = "Decelerating";
            } else if (Math.abs(s.velocity) > 0.01) {
                phase = "Constant speed";
            } else {
                phase = "At rest";
            }

            System.out.printf("%.2f     %8.3f     %8.3f     %8.3f     %s%n", t, s.position, s.velocity, s.acceleration,
                    phase);
        }

        System.out.println("\nProfile shape:");
        System.out.println("1. Accelerate to max velocity (or until midpoint)");
        System.out.println("2. Coast at max velocity (if distance allows)");
        System.out.println("3. Decelerate to rest at target");
    }
}
