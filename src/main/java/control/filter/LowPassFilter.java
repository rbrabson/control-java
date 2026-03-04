package control.filter;

public class LowPassFilter implements Filter {
    private double gain;
    private double previousEstimate;
    private boolean initialized;

    public LowPassFilter(double gain) {
        if (gain <= 0 || gain >= 1) {
            throw new IllegalArgumentException("gain must be between 0 and 1 (exclusive)");
        }
        this.gain = gain;
        this.previousEstimate = 0.0;
        this.initialized = false;
    }

    @Override
    public double estimate(double measurement) {
        if (!initialized) {
            previousEstimate = measurement;
            initialized = true;
            return measurement;
        }

        double estimate = gain * previousEstimate + (1 - gain) * measurement;
        previousEstimate = estimate;
        return estimate;
    }

    @Override
    public double getGain() {
        return gain;
    }

    public void setGain(double gain) {
        if (gain <= 0 || gain >= 1) {
            throw new IllegalArgumentException("gain must be between 0 and 1 (exclusive)");
        }
        this.gain = gain;
    }

    @Override
    public void reset() {
        previousEstimate = 0.0;
        initialized = false;
    }

    public double getLastEstimate() {
        return initialized ? previousEstimate : 0.0;
    }

    public boolean isInitialized() {
        return initialized;
    }
}
