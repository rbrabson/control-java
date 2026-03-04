package control.examples.motionprofile.basic;

import control.motionprofile.Constraints;
import control.motionprofile.MotionProfile;
import control.motionprofile.State;

public class Main {
    public static void main(String[] args) {
        MotionProfile profile = new MotionProfile(new Constraints(2.0, 1.0), new State(0.0, 0.0, 0.0, 0.0),
                new State(5.0, 0.0, 0.0, 0.0));

        for (double t = 0.0; t <= profile.totalTime(); t += 0.5) {
            State s = profile.calculate(t);
            System.out.printf("t=%.2f pos=%.3f vel=%.3f acc=%.3f%n", t, s.position, s.velocity, s.acceleration);
        }
    }
}
