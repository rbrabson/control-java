package control.motionprofile;

public class MotionProfile {
    private final Constraints constraints;
    private final State initial;
    private final State goal;

    private double accelerationTime;
    private double cruiseTime;
    private double decelerationTime;
    private double totalTime;

    private double cruiseVelocity;
    private double accelerationDistance;
    private double cruiseDistance;

    public MotionProfile(Constraints constraints, State initial, State goal) {
        this.constraints = constraints;
        this.initial = initial;
        this.goal = goal;
        calculateProfile();
    }

    private void calculateProfile() {
        double displacement = goal.position - initial.position;

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

    public boolean isFinished(double t) {
        return t >= totalTime;
    }

    public double totalTime() {
        return totalTime;
    }

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
