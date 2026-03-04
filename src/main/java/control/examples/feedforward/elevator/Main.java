package control.examples.feedforward.elevator;

import control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        FeedForward elevatorFF = new FeedForward(0.0, 0.9, 0.2, FeedForward.withGravityGain(9.81));

        double output = elevatorFF.calculate(0.0, 1.0, 0.0);
        System.out.printf("Elevator FF output: %.3f%n", output);
    }
}
