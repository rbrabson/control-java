package control.motionprofile;

/**
 * A motion profile that generates a trajectory from an initial state to a goal
 * state while respecting specified constraints on maximum velocity and
 * acceleration. The profile consists of three phases: acceleration, cruising at
 * constant velocity, and deceleration. The profile can be queried at any time
 * to get the current state (position, velocity, acceleration) of the
 * trajectory.
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
        this.constraints = constraints;
        this.initial = initial;
        this.goal = goal;
        calculateProfile();
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
        if (Math.abs(displacement) < 1e-10) {
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

        double accelDist = (maxVel * maxVel - vStart * vStart) / (2 * constraints.maxAcceleration * direction);
        double decelDist = (vEnd * vEnd - maxVel * maxVel) / (-2 * constraints.maxAcceleration * direction);

        if (direction * (accelDist + decelDist) <= direction * displacement) {
            accelerationTime = (maxVel - vStart) / (constraints.maxAcceleration * direction);
            decelerationTime = (vEnd - maxVel) / (-constraints.maxAcceleration * direction);
            cruiseVelocity = maxVel;
            accelerationDistance = accelDist;
            cruiseDistance = displacement - accelDist - decelDist;
            cruiseTime = cruiseDistance / maxVel;
        } else {
            double discriminant = vStart * vStart + vEnd * vEnd
                    + 2 * constraints.maxAcceleration * displacement * direction;
            if (discriminant < 0) {
                cruiseVelocity = vStart;
                totalTime = 0;
                return;
            }

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
        if (t <= 0) {
            return initial;
        }
        if (t >= totalTime) {
            return goal;
        }

        double direction = goal.position < initial.position ? -1.0 : 1.0;

        double position;
        double velocity;
        double acceleration;

        if (t <= accelerationTime) {
            acceleration = constraints.maxAcceleration * direction;
            velocity = initial.velocity + acceleration * t;
            position = initial.position + initial.velocity * t + 0.5 * acceleration * t * t;
        } else if (t <= accelerationTime + cruiseTime) {
            acceleration = 0;
            velocity = cruiseVelocity;
            double cruiseT = t - accelerationTime;
            position = initial.position + accelerationDistance + cruiseVelocity * cruiseT;
        } else {
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
        double direction = goal.position < initial.position ? -1.0 : 1.0;

        double targetDistance = targetPosition - initial.position;
        if (direction * targetDistance <= 0) {
            return 0;
        }

        double totalDistance = goal.position - initial.position;
        if (direction * targetDistance >= direction * totalDistance) {
            return totalTime;
        }

        if (direction * targetDistance <= direction * accelerationDistance) {
            double a = 0.5 * constraints.maxAcceleration * direction;
            double b = initial.velocity;
            double c = -targetDistance;
            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0) {
                return 0;
            }
            return (-b + Math.sqrt(discriminant)) / (2 * a);
        } else if (direction * targetDistance <= direction * (accelerationDistance + cruiseDistance)) {
            double cDistance = targetDistance - accelerationDistance;
            return accelerationTime + cDistance / cruiseVelocity;
        } else {
            double remainingDistance = totalDistance - targetDistance;
            double a = 0.5 * constraints.maxAcceleration * direction;
            double b = goal.velocity;
            double c = remainingDistance;
            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0) {
                return totalTime;
            }
            double timeFromEnd = (-b + Math.sqrt(discriminant)) / (2 * a);
            return totalTime - timeFromEnd;
        }
    }
}
