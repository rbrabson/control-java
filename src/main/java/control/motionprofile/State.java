package control.motionprofile;

public class State {
    public final double position;
    public final double velocity;
    public final double acceleration;
    public final double time;

    public State(double position, double velocity, double acceleration, double time) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.time = time;
    }
}
