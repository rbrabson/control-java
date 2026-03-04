package control.examples.motionprofile.fullstate_control;

import control.feedback.FullStateFeedback;
import control.motionprofile.Constraints;
import control.motionprofile.MotionProfile;
import control.motionprofile.State;

public class Main {
    public static void main(String[] args) {
        MotionProfile profile = new MotionProfile(new Constraints(3.0, 2.0), new State(0.0, 0.0, 0.0, 0.0),
                new State(4.0, 0.0, 0.0, 0.0));

        FullStateFeedback fsf = new FullStateFeedback(new double[] { 2.0, 0.4 });
        double[] measured = { 0.0, 0.0 };

        for (double t = 0.0; t <= profile.totalTime(); t += 0.2) {
            State desired = profile.calculate(t);
            double output = fsf.calculate(new double[] { desired.position, desired.velocity }, measured);
            measured[1] += output * 0.02;
            measured[0] += measured[1] * 0.02;
            System.out.printf("t=%.2f desiredPos=%.2f measPos=%.2f out=%.3f%n", t, desired.position, measured[0],
                    output);
        }
    }
}
