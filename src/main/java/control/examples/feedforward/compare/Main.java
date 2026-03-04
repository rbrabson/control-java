package control.examples.feedforward.compare;

import control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        FeedForward basic = new FeedForward(0.0, 1.0, 0.2);
        FeedForward elevator = new FeedForward(0.0, 1.0, 0.2, FeedForward.withGravityGain(9.81));
        FeedForward arm = new FeedForward(0.0, 1.0, 0.2, FeedForward.withCosineGain(2.5));

        double position = Math.PI / 6.0;
        double velocity = 1.0;
        double accel = 0.3;

        System.out.printf("Basic: %.3f%n", basic.calculate(position, velocity, accel));
        System.out.printf("Elevator: %.3f%n", elevator.calculate(position, velocity, accel));
        System.out.printf("Arm: %.3f%n", arm.calculate(position, velocity, accel));
    }
}
