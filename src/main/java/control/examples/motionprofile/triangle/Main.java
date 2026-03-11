package control.examples.motionprofile.triangle;

import control.motionprofile.Constraints;
import control.motionprofile.MotionProfile;
import control.motionprofile.State;

public class Main {
    public static void main(String[] args) {
        // Triangle profile: distance too short to reach max velocity
        // High max velocity (10.0) but limited by acceleration and short distance
        MotionProfile shortProfile = new MotionProfile(new Constraints(10.0, 2.0), new State(0.0, 0.0, 0.0, 0.0),
                new State(1.0, 0.0, 0.0, 0.0));

        // Trapezoidal profile: distance long enough to reach max velocity
        MotionProfile longProfile = new MotionProfile(new Constraints(10.0, 2.0), new State(0.0, 0.0, 0.0, 0.0),
                new State(20.0, 0.0, 0.0, 0.0));

        System.out.println("Triangle vs Trapezoidal Motion Profiles");
        System.out.println("========================================\n");

        // Analyze short profile
        System.out.println("SHORT DISTANCE (1 unit):");
        System.out.printf("Total time: %.3f s%n", shortProfile.totalTime());
        State midShort = shortProfile.calculate(shortProfile.totalTime() / 2.0);
        System.out.printf("Peak velocity at midpoint: %.3f m/s (%.1f%% of max)%n", midShort.velocity,
                (midShort.velocity / 10.0) * 100);
        System.out.println("Profile shape: TRIANGLE (accelerate then decelerate, no cruising)\n");

        // Analyze long profile
        System.out.println("LONG DISTANCE (20 units):");
        System.out.printf("Total time: %.3f s%n", longProfile.totalTime());
        State midLong = longProfile.calculate(longProfile.totalTime() / 2.0);
        System.out.printf("Velocity at midpoint: %.3f m/s (%.1f%% of max)%n", midLong.velocity,
                (midLong.velocity / 10.0) * 100);
        System.out.println("Profile shape: TRAPEZOID (accelerate, cruise at max, decelerate)\n");

        System.out.println("Key insight:");
        System.out.println("- Triangle: Short moves never reach max velocity");
        System.out.println("- Trapezoid: Long moves include constant-velocity cruising phase");
        System.out.println("- Transition depends on: distance, max velocity, and max acceleration");
    }
}
