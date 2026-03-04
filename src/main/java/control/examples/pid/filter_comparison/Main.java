package control.examples.pid.filter_comparison;

import control.filter.KalmanFilter;
import control.filter.LowPassFilter;
import control.pid.PID;

public class Main {
    public static void main(String[] args) {
        PID lowPassPID = new PID(0.5, 0.1, 0.05, PID.withFilter(new LowPassFilter(0.5)),
                PID.withOutputLimits(-100, 100));

        PID kalmanPID = new PID(0.5, 0.1, 0.05, PID.withFilter(new KalmanFilter(0.05, 0.1, 10)),
                PID.withOutputLimits(-100, 100));

        PID noFilterPID = new PID(0.5, 0.1, 0.05, PID.withOutputLimits(-100, 100));

        double setpoint = 50.0;
        double x1 = 10.0;
        double x2 = 10.0;
        double x3 = 10.0;

        for (int i = 0; i < 200; i++) {
            x1 += lowPassPID.calculate(setpoint, x1) * 0.01;
            x2 += kalmanPID.calculate(setpoint, x2) * 0.01;
            x3 += noFilterPID.calculate(setpoint, x3) * 0.01;
        }

        System.out.printf("LowPass=%.2f Kalman=%.2f NoFilter=%.2f%n", x1, x2, x3);
    }
}
