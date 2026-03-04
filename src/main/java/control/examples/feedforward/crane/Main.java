package control.examples.feedforward.crane;

import control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        FeedForward craneFF = new FeedForward(0.0, 1.1, 0.25, FeedForward.withGravityGain(15.7),
                FeedForward.withCosineGain(8.2));

        double output = craneFF.calculate(Math.PI / 4.0, 0.8, 0.1);
        System.out.printf("Crane FF output: %.3f%n", output);
    }
}
