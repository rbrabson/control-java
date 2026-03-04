package control.examples.motionprofile.triangle;

import control.motionprofile.Constraints;
import control.motionprofile.MotionProfile;
import control.motionprofile.State;

public class Main {
    public static void main(String[] args) {
        MotionProfile profile = new MotionProfile(new Constraints(10.0, 2.0), new State(0.0, 0.0, 0.0, 0.0),
                new State(1.0, 0.0, 0.0, 0.0));

        System.out.printf("Triangle-like profile total time: %.3f s%n", profile.totalTime());
        State midpoint = profile.calculate(profile.totalTime() / 2.0);
        System.out.printf("Midpoint pos=%.3f vel=%.3f%n", midpoint.position, midpoint.velocity);
    }
}
