package control.filter;

import java.util.Arrays;

public class LinearRegression {
    private double[] data;
    private double slope;
    private double intercept;
    private boolean hasRun;

    public LinearRegression(double[] data) {
        this.data = Arrays.copyOf(data, data.length);
    }

    public void runLeastSquares() {
        int n = data.length;
        if (n < 2) {
            slope = 0;
            intercept = 0;
            hasRun = true;
            return;
        }

        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumXX = 0.0;

        for (int i = 0; i < data.length; i++) {
            double x = i;
            double y = data[i];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
        }

        double nf = n;
        double denominator = nf * sumXX - sumX * sumX;
        if (Math.abs(denominator) < 1e-10) {
            slope = 0;
            intercept = n > 0 ? sumY / nf : 0;
        } else {
            slope = (nf * sumXY - sumX * sumY) / denominator;
            intercept = (sumY - slope * sumX) / nf;
        }

        hasRun = true;
    }

    public double predictNextValue() {
        if (!hasRun) {
            runLeastSquares();
        }

        if (data.length == 1) {
            return data[0];
        }

        double nextX = data.length;
        return slope * nextX + intercept;
    }

    public void updateData(double[] data) {
        this.data = Arrays.copyOf(data, data.length);
        this.hasRun = false;
    }
}
