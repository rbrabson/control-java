package control.motionprofile;

public class Constraints {
    public final double maxVelocity;
    public final double maxAcceleration;

    public Constraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
}
