package control.feedforward;

import java.util.function.Consumer;

public class FeedForward {
    public interface Option extends Consumer<FeedForward> {
    }

    private final double kS;
    private final double kV;
    private final double kA;
    private double kG;
    private double kCos;

    public FeedForward(double kS, double kV, double kA, Option... options) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = 0.0;
        this.kCos = 0.0;

        for (Option option : options) {
            option.accept(this);
        }
    }

    public static Option withGravityGain(double kG) {
        return ff -> ff.kG = kG;
    }

    public static Option withCosineGain(double kCos) {
        return ff -> ff.kCos = kCos;
    }

    public double calculate(double position, double velocity, double acceleration) {
        double output = kV * velocity + kA * acceleration + kG;
        if (kCos != 0.0) {
            output += kCos * Math.cos(position);
        }
        return output;
    }

    public double getKS() {
        return kS;
    }

    public double getKV() {
        return kV;
    }

    public double getKA() {
        return kA;
    }
}
