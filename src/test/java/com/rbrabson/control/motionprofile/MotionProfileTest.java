package com.rbrabson.control.motionprofile;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

class MotionProfileTest {
    @Test
    void reachesGoalAtTotalTime() {
        Constraints constraints = new Constraints(2.0, 1.0);
        State initial = new State(0.0, 0.0, 0.0, 0.0);
        State goal = new State(5.0, 0.0, 0.0, 0.0);

        MotionProfile profile = new MotionProfile(constraints, initial, goal);
        State end = profile.calculate(profile.totalTime());

        assertTrue(Math.abs(end.position - 5.0) < 1e-9);
        assertTrue(profile.isFinished(profile.totalTime()));
    }
}
