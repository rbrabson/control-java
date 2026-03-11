package com.rbrabson.control.motionprofile;

/**
 * Represents the constraints for a motion profile, including maximum velocity
 * and maximum acceleration.
 */
public class Constraints {
    public final double maxVelocity;
    public final double maxAcceleration;

    /**
     * Creates a new Constraints object with the specified maximum velocity and
     * maximum acceleration.
     *
     * @param maxVelocity     The maximum velocity for the motion profile.
     * @param maxAcceleration The maximum acceleration for the motion profile.
     */
    public Constraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
}
