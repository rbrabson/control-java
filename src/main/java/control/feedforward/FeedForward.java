package control.feedforward;

import java.util.function.Consumer;

/**
 * A feedforward controller for calculating the necessary motor output to
 * achieve a desired velocity and acceleration. The controller can be configured
 * with static, velocity, and acceleration gains, as well as optional gravity
 * and cosine gains for more complex systems.
 */
public class FeedForward {
    /**
     * An option interface for configuring the feedforward controller with
     * additional gains or settings. This allows for a flexible and extensible way
     * to customize the behavior of the feedforward controller without needing to
     * create multiple constructors or subclasses.
     */
    public interface Option extends Consumer<FeedForward> {
    }

    private final double kS;
    private final double kV;
    private final double kA;
    private double kG;
    private double kCos;

    /**
     * Creates a new FeedForward controller with the specified gains and options.
     *
     * @param kS      The static gain, representing the minimum output required to
     *                overcome static friction.
     * @param kV      The velocity gain, representing the output required to
     *                maintain a certain velocity.
     * @param kA      The acceleration gain, representing the output required to
     *                achieve a certain acceleration.
     * @param options Optional configuration options for the feedforward controller,
     *                such as gravity and cosine gains.
     */
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

    /**
     * Creates an option to set the gravity gain for the feedforward controller.
     * This gain can be used to compensate for the effect of gravity on the system,
     * such as when controlling an arm or elevator.
     *
     * @param kG The gravity gain, representing the output required to compensate
     *           for gravity.
     * @return An option that can be passed to the FeedForward constructor to set
     *         the gravity gain.
     */

    public static Option withGravityGain(double kG) {
        return ff -> ff.kG = kG;
    }

    /**
     * Creates an option to set the cosine gain for the feedforward controller. This
     * gain can be used to compensate for the effect of position on the system, such
     * as when controlling an arm or elevator that experiences varying loads at
     * different positions.
     *
     * @param kCos The cosine gain, representing the output required to compensate
     *             for position.
     * @return An option that can be passed to the FeedForward constructor to set
     *         the cosine gain.
     */

    public static Option withCosineGain(double kCos) {
        return ff -> ff.kCos = kCos;
    }

    /**
     * Calculates the feedforward output based on the current position, velocity,
     * and acceleration. The output is calculated using the formula:
     *
     * @param position     The current position of the system, which can be used to
     *                     calculate the effect of position on the output if a
     *                     cosine gain is configured.
     * @param velocity     The desired velocity of the system, which is multiplied
     *                     by the velocity gain to calculate the velocity component
     *                     of the output.
     * @param acceleration The desired acceleration of the system, which is
     *                     multiplied by the acceleration gain to calculate the
     *                     acceleration component of the output.
     * @return The calculated feedforward output, which can be applied to the motors
     *         to achieve the desired velocity and acceleration while compensating
     *         for static friction, gravity, and position effects as configured.
     */
    public double calculate(double position, double velocity, double acceleration) {
        double output = kV * velocity + kA * acceleration + kG;
        if (kCos != 0.0) {
            output += kCos * Math.cos(position);
        }
        return output;
    }

    /**
     * Gets the static gain of the feedforward controller, which represents the
     * minimum output required to overcome static friction.
     *
     * @return The static gain of the feedforward controller.
     */
    public double getKS() {
        return kS;
    }

    /**
     * Gets the velocity gain of the feedforward controller, which represents the
     * output required to maintain a certain velocity.
     *
     * @return The velocity gain of the feedforward controller.
     */
    public double getKV() {
        return kV;
    }

    /**
     * Gets the acceleration gain of the feedforward controller, which represents
     * the output required to achieve a certain acceleration.
     *
     * @return The acceleration gain of the feedforward controller.
     */
    public double getKA() {
        return kA;
    }
}
