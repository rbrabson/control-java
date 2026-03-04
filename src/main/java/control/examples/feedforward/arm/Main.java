package control.examples.feedforward.arm;

import control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        FeedForward armFF = new FeedForward(0.0, 1.0, 0.2, FeedForward.withCosineGain(2.5));

        double angleRad = Math.PI / 3.0;
        double output = armFF.calculate(angleRad, 1.5, 0.5);
        System.out.printf("Arm FF (angle=%.2f rad) output: %.3f%n", angleRad, output);
    }
}
