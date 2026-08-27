package com.rbrabson.control.motionprofile;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class MotionProfileTest {
    @Test
    void reachesGoalAtTotalTime() {
        Constraints constraints = new Constraints(2.0, 1.0);
        State initial = new State(0.0, 0.0, 0.0, 0.0);
        State goal = new State(5.0, 0.0, 0.0, 0.0);

        MotionProfile profile = new MotionProfile(constraints, initial, goal);
        State end = profile.calculate(profile.totalTime());

        assertTrue(Math.abs(end.position - 5.0) < 1e-9);
        assertEquals(profile.totalTime(), end.time, 1e-9);
        assertTrue(profile.isFinished(profile.totalTime()));
    }

    @Test
    void supportsReverseMotionAndTimeLookup() {
        Constraints constraints = new Constraints(2.0, 1.0);
        MotionProfile profile = new MotionProfile(constraints,
                new State(5.0, 0.0, 0.0, 0.0), new State(0.0, 0.0, 0.0, 0.0));

        assertEquals(3.0, profile.calculate(profile.timeLeftUntil(3.0)).position, 1e-9);
        assertEquals(profile.totalTime(), profile.timeLeftUntil(0.0), 1e-9);
    }

    @Test
    void rejectsEndpointVelocityOppositeToMotion() {
        assertThrows(IllegalArgumentException.class, () -> new MotionProfile(new Constraints(3.0, 2.0),
                new State(0.0, -1.0, 0.0, 0.0), new State(5.0, 1.0, 0.0, 0.0)));
    }

    @Test
    void rejectsInvalidConstraintsTimesAndUnsupportedZeroDisplacementChange() {
        assertThrows(IllegalArgumentException.class, () -> new Constraints(0.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new Constraints(1.0, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new MotionProfile(null,
                new State(0.0, 0.0, 0.0, 0.0), new State(1.0, 0.0, 0.0, 0.0)));

        Constraints constraints = new Constraints(2.0, 1.0);
        assertThrows(IllegalArgumentException.class, () -> new MotionProfile(constraints,
                new State(0.0, 1.0, 0.0, 0.0), new State(0.0, 0.0, 0.0, 0.0)));

        MotionProfile profile = new MotionProfile(constraints,
                new State(0.0, 0.0, 0.0, 0.0), new State(1.0, 0.0, 0.0, 0.0));
        assertEquals(0.0, profile.calculate(Double.NaN).time, 1e-9);
        assertThrows(IllegalArgumentException.class, () -> new MotionProfile(constraints,
                new State(0.0, 3.0, 0.0, 0.0), new State(1.0, 0.0, 0.0, 0.0)));
        assertEquals(0.0, profile.timeLeftUntil(Double.NaN), 1e-9);
        assertEquals(0.0, profile.calculate(-1.0).time, 1e-9);
        assertEquals(profile.totalTime(), profile.calculate(profile.totalTime() + 1.0).time, 1e-9);
    }
}
