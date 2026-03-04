package control.examples.pid.dampening;

import control.filter.LowPassFilter;
import control.pid.PID;

public class Main {
    public static void main(String[] args) {
        PID basic = new PID(2.0, 0.5, 0.8, PID.withOutputLimits(-50, 50));
        PID filtered = new PID(2.0, 0.5, 0.8, PID.withFilter(new LowPassFilter(0.4)), PID.withOutputLimits(-50, 50));
        PID damped = new PID(2.0, 0.5, 0.8, PID.withFilter(new LowPassFilter(0.4)), PID.withStabilityThreshold(3.0),
                PID.withOutputLimits(-50, 50));

        double target = 100.0;
        double x1 = 20.0;
        double x2 = 20.0;
        double x3 = 20.0;

        for (int i = 0; i < 50; i++) {
            double u1 = basic.calculate(target, x1);
            double u2 = filtered.calculate(target, x2);
            double u3 = damped.calculate(target, x3);
            x1 += u1 * 0.01;
            x2 += u2 * 0.01;
            x3 += u3 * 0.01;
        }

        System.out.printf("Final basic=%.2f filtered=%.2f damped=%.2f%n", x1, x2, x3);
    }
}
