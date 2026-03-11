package control.pid;

import control.filter.LowPassFilter;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

class PIDTest {
    @Test
    void respectsOutputLimits() {
        PID pid = new PID(10.0, 0.0, 0.0, PID.withOutputLimits(-5.0, 5.0));
        double out = pid.calculate(10.0, 0.0);
        assertTrue(out <= 5.0 && out >= -5.0);
    }

    @Test
    void supportsDerivativeFilterOption() {
        PID pid = new PID(0.0, 0.0, 1.0, PID.withFilter(new LowPassFilter(0.8)));
        pid.calculate(10.0, 0.0);
        double out = pid.calculate(8.0, 0.0);
        assertTrue(Double.isFinite(out));
    }
}
