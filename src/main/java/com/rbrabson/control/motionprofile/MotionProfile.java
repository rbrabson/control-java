package com.rbrabson.control.motionprofile;

/**
 * A motion profile that generates a trajectory from an initial state to a goal
 * state while respecting specified constraints on maximum velocity and
 * acceleration. The profile consists of three phases: acceleration, cruising at
 * constant velocity, and deceleration, and can pass through a velocity reversal
 * when required by the endpoint velocities. The profile can be queried at any
 * time to get the current state (position, velocity, acceleration) of the
 * trajectory. The acceleration fields supplied in the endpoint states are
 * metadata; the profile uses its configured maximum acceleration.
 */
public class MotionProfile {
    private final Constraints constraints;
    private final State initial;
    private final State goal;

    private double accelerationTime;
    private double cruiseTime;
    private double totalTime;

    private double cruiseVelocity;
    private double accelerationDistance;
    private double cruiseDistance;

    /**
     * Constructs a motion profile given the constraints, initial state, and goal
     * state.
     *
     * @param constraints The constraints on maximum velocity and acceleration.
     * @param initial     The initial state of the trajectory (position, velocity).
     * @param goal        The goal state of the trajectory (position, velocity).
     */
    public MotionProfile(Constraints constraints, State initial, State goal) {
        if (constraints == null || initial == null || goal == null) {
            throw new IllegalArgumentException("constraints and states must be non-null");
        }
        this.constraints = constraints;
        this.initial = initial;
        this.goal = goal;
        validateState(initial);
        validateState(goal);
        calculateProfile();
    }

    private static void validateState(State state) {
        if (!Double.isFinite(state.position) || !Double.isFinite(state.velocity)) {
            throw new IllegalArgumentException("state position and velocity must be finite");
        }
    }

    /**
     * Calculates the motion profile based on the initial and goal states and the
     * constraints. This method determines the time spent in each phase of the
     * motion (acceleration, cruising, deceleration) and the corresponding distances
     * and velocities for each phase.
     */
    private void calculateProfile() {
        double displacement = goal.position - initial.position;
        double decelerationTime;

        // Handle the case where there is no displacement
        if (Math.abs(displacement) < 1e-10) {
            if (Math.abs(goal.velocity - initial.velocity) >= 1e-10) {
                throw new IllegalArgumentException(
                        "a zero-displacement profile cannot change velocity");
            }
            totalTime = Math.abs(goal.velocity - initial.velocity) / constraints.maxAcceleration;
            accelerationTime = totalTime;
            cruiseTime = 0;
            decelerationTime = 0;
            cruiseVelocity = goal.velocity;
            return;
        }

        double direction = displacement < 0 ? -1.0 : 1.0;
        double maxVel = constraints.maxVelocity * direction;

        double vStart = initial.velocity;
        double vEnd = goal.velocity;
        if (Math.abs(vStart) > constraints.maxVelocity
                || Math.abs(vEnd) > constraints.maxVelocity) {
            throw new IllegalArgumentException(
                    "initial and goal velocities must respect max velocity");
        }

        double accelDist = (maxVel * maxVel - vStart * vStart) / (2 * constraints.maxAcceleration * direction);
        double decelDist = (vEnd * vEnd - maxVel * maxVel) / (-2 * constraints.maxAcceleration * direction);

        if (direction * (accelDist + decelDist) <= direction * displacement) {
            // There is a cruising phase
            accelerationTime = (maxVel - vStart) / (constraints.maxAcceleration * direction);
            decelerationTime = (vEnd - maxVel) / (-constraints.maxAcceleration * direction);
            cruiseVelocity = maxVel;
            accelerationDistance = accelDist;
            cruiseDistance = displacement - accelDist - decelDist;
            cruiseTime = cruiseDistance / maxVel;
        } else {
            // No cruising phase; triangle profile
            double discriminant = vStart * vStart + vEnd * vEnd
                    + 2 * constraints.maxAcceleration * displacement * direction;
            if (discriminant < 0) {
                throw new IllegalArgumentException("initial and goal states are unreachable");
            }

            // Calculate peak velocity
            double peakVel = direction * Math.sqrt(discriminant / 2);
            cruiseVelocity = peakVel;
            accelerationTime = (peakVel - vStart) / (constraints.maxAcceleration * direction);
            decelerationTime = (vEnd - peakVel) / (-constraints.maxAcceleration * direction);
            cruiseTime = 0;
            accelerationDistance = (peakVel * peakVel - vStart * vStart)
                    / (2 * constraints.maxAcceleration * direction);
            cruiseDistance = 0;
        }

        totalTime = accelerationTime + cruiseTime + decelerationTime;
        if (accelerationTime < -1e-10 || cruiseTime < -1e-10 || decelerationTime < -1e-10
                || !Double.isFinite(totalTime)) {
            throw new IllegalArgumentException("initial and goal states are unsupported or unreachable");
        }
    }

    /**
     * Calculates the state of the motion profile at a given time t. This method
     * determines which phase of the motion the profile is in at time t
     * (acceleration, cruising, or deceleration) and calculates the corresponding
     * position, velocity, and acceleration based on the equations of motion for
     * that phase.
     *
     * @param t The time at which to calculate the state of the motion profile.
     * @return The state (position, velocity, acceleration) of the motion profile at
     *         time t.
     */
    public State calculate(double t) {
        if (!Double.isFinite(t)) {
            throw new IllegalArgumentException("time must be finite");
        }
        if (t <= 0) {
            return new State(initial.position, initial.velocity, initial.acceleration, t);
        }
        if (t >= totalTime) {
            return new State(goal.position, goal.velocity, goal.acceleration, t);
        }

        double direction = goal.position < initial.position ? -1.0 : 1.0;

        double position;
        double velocity;
        double acceleration;

        if (t <= accelerationTime) {
            // Acceleration phase
            acceleration = constraints.maxAcceleration * direction;
            velocity = initial.velocity + acceleration * t;
            position = initial.position + initial.velocity * t + 0.5 * acceleration * t * t;
        } else if (t <= accelerationTime + cruiseTime) {
            // Cruising phase
            acceleration = 0;
            velocity = cruiseVelocity;
            double cruiseT = t - accelerationTime;
            position = initial.position + accelerationDistance + cruiseVelocity * cruiseT;
        } else {
            // Deceleration phase
            double decelT = t - accelerationTime - cruiseTime;
            acceleration = -constraints.maxAcceleration * direction;
            velocity = cruiseVelocity + acceleration * decelT;
            position = initial.position + accelerationDistance + cruiseDistance + cruiseVelocity * decelT
                    + 0.5 * acceleration * decelT * decelT;
        }

        return new State(position, velocity, acceleration, t);
    }

    /**
     * Determines if the motion profile has finished at a given time t. The profile
     * is considered finished if time t is greater than or equal to the total time
     * of the profile.
     *
     * @param t The time at which to check if the motion profile is finished.
     * @return True if the motion profile is finished at time t, false otherwise.
     */
    public boolean isFinished(double t) {
        if (!Double.isFinite(t)) {
            throw new IllegalArgumentException("time must be finite");
        }
        return t >= totalTime;
    }

    /**
     * Returns the total time of the motion profile, which is the sum of the time
     * spent in acceleration, cruising, and deceleration phases.
     *
     * @return The total time of the motion profile.
     */
    public double totalTime() {
        return totalTime;
    }

    /**
     * Calculates the time left until the profile reaches a specified target
     * position. This method determines how much time is remaining until the profile
     * reaches the target position based on the current phase of the motion and the
     * equations of motion for that phase.
     *
     * @param targetPosition The target position to calculate the time left until.
     * @return The time left until the profile reaches the target position.
     */
    public double timeLeftUntil(double targetPosition) {
        if (!Double.isFinite(targetPosition)) {
            throw new IllegalArgumentException("target position must be finite");
        }
        double direction = goal.position < initial.position ? -1.0 : 1.0;

        double targetDistance = targetPosition - initial.position;
        if (direction * targetDistance <= 0) {
            return 0;
        }

        double totalDistance = goal.position - initial.position;
        if (direction * targetDistance >= direction * totalDistance) {
            return totalTime;
        }

        double targetAlongPath = direction * targetDistance;
        double accelerationAlongPath = direction * accelerationDistance;
        double cruiseEndAlongPath = direction * (accelerationDistance + cruiseDistance);

        if (targetAlongPath <= accelerationAlongPath) {
            double initialVelocityAlongPath = direction * initial.velocity;
            double discriminant = initialVelocityAlongPath * initialVelocityAlongPath
                    + 2 * constraints.maxAcceleration * targetAlongPath;
            return (-initialVelocityAlongPath + Math.sqrt(Math.max(0, discriminant)))
                    / constraints.maxAcceleration;
        } else if (targetAlongPath <= cruiseEndAlongPath) {
            double cDistance = targetAlongPath - accelerationAlongPath;
            return accelerationTime + cDistance / (direction * cruiseVelocity);
        } else {
            double remainingDistance = direction * (totalDistance - targetDistance);
            double goalVelocityAlongPath = direction * goal.velocity;
            double discriminant = goalVelocityAlongPath * goalVelocityAlongPath
                    + 2 * constraints.maxAcceleration * remainingDistance;
            double timeFromEnd = (-goalVelocityAlongPath + Math.sqrt(Math.max(0, discriminant)))
                    / constraints.maxAcceleration;
            return totalTime - timeFromEnd;
        }
    }
}
