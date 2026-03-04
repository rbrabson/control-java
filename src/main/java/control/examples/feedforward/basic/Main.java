package control.examples.feedforward.basic;

import control.feedforward.FeedForward;

public class Main {
    public static void main(String[] args) {
        FeedForward ff = new FeedForward(0.0, 1.2, 0.3);
        double output = ff.calculate(0.0, 2.0, 1.0);
        System.out.printf("Basic FF output: %.3f%n", output);
    }
}
