package com.rbrabson.control.motionprofile;

/**
 * Represents the state of a motion profile at a given time, including position,
 * velocity, acceleration, and time. This class is used to define the desired
 * state of the robot at specific points along a motion profile trajectory.
 */
public class State {
    public final double position;
    public final double velocity;
    public final double acceleration;
    public final double time;

    /**
     * Constructs a new State with the specified position, velocity, acceleration,
     * and time.
     *
     * @param position     The position of the robot at this state.
     * @param velocity     The velocity of the robot at this state.
     * @param acceleration The acceleration of the robot at this state.
     * @param time         The time at which this state occurs in the motion
     *                     profile.
     */
    public State(double position, double velocity, double acceleration, double time) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.time = time;
    }
}
