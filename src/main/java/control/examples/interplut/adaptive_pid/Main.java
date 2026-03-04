package control.examples.interplut.adaptive_pid;

import control.interplut.InterpLUT;
import control.pid.PID;

import static control.interplut.InterpLUT.add;

public class Main {
    public static void main(String[] args) {
        InterpLUT kpLut = new InterpLUT(add(0, 0.6), add(20, 0.8), add(50, 1.2), add(100, 1.6));

        double setpoint = 75.0;
        double state = 10.0;
        double errorMagnitude = Math.abs(setpoint - state);

        double adaptiveKp = kpLut.get(errorMagnitude);
        PID controller = new PID(adaptiveKp, 0.05, 0.02, PID.withOutputLimits(-1, 1));

        double output = controller.calculate(setpoint, state);
        System.out.printf("Adaptive Kp=%.3f, output=%.3f%n", adaptiveKp, output);
    }
}
