package control.filter;

public class KalmanFilter implements Filter {
    private final double q;
    private final double r;
    private final int n;
    private double p;
    private double k;
    private double x;
    private SizedStack<Double> estimates;
    private LinearRegression regression;

    public KalmanFilter(double q, double r, int n) {
        if (n <= 0) {
            throw new IllegalArgumentException("stack size must be positive");
        }
        if (q < 0 || r < 0) {
            throw new IllegalArgumentException("covariance values must be non-negative");
        }

        this.q = q;
        this.r = r;
        this.n = n;
        this.p = 1.0;
        this.k = 0.0;
        this.x = 0.0;
        this.estimates = new SizedStack<>(n);
        reset();

        for (int i = 0; i < n; i++) {
            this.estimates.push(0.0);
        }

        this.regression = new LinearRegression(toPrimitive(estimates));
        findK();
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getX() {
        return x;
    }

    public double getK() {
        return k;
    }

    public double getP() {
        return p;
    }

    @Override
    public double getGain() {
        return k;
    }

    @Override
    public double estimate(double measurement) {
        regression.runLeastSquares();
        double prediction = regression.predictNextValue();

        Double latest = estimates.peek();
        double lastEstimate = latest == null ? 0.0 : latest;

        x += prediction - lastEstimate;
        x += k * (measurement - x);

        estimates.push(x);
        regression.updateData(toPrimitive(estimates));

        return x;
    }

    private void findK() {
        for (int i = 0; i < 2000; i++) {
            solveDARE();
        }
    }

    private void solveDARE() {
        p += q;
        k = p / (p + r);
        p = (1 - k) * p;
    }

    @Override
    public void reset() {
        double convergedP = p;
        double convergedK = k;

        x = 0.0;
        estimates = new SizedStack<>(n);
        for (int i = 0; i < n; i++) {
            estimates.push(0.0);
        }

        regression = new LinearRegression(toPrimitive(estimates));

        p = convergedP;
        k = convergedK;
    }

    private static double[] toPrimitive(SizedStack<Double> stack) {
        var values = stack.toList();
        double[] out = new double[values.size()];
        for (int i = 0; i < values.size(); i++) {
            out[i] = values.get(i);
        }
        return out;
    }
}
