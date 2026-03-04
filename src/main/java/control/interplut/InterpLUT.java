package control.interplut;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class InterpLUT {
    private final List<Double> x;
    private final List<Double> y;
    private List<Double> m;

    public InterpLUT() {
        this.x = new ArrayList<>();
        this.y = new ArrayList<>();
        this.m = new ArrayList<>();
    }

    public InterpLUT add(double input, double output) {
        x.add(input);
        y.add(output);
        return this;
    }

    public InterpLUT createLUT() {
        if (x.size() != y.size() || x.size() < 2) {
            throw new IllegalStateException(
                    "there must be at least two control points and the arrays must be of equal length");
        }

        List<Point> points = new ArrayList<>(x.size());
        for (int i = 0; i < x.size(); i++) {
            points.add(new Point(x.get(i), y.get(i)));
        }
        points.sort(Comparator.comparingDouble(p -> p.x));

        int n = points.size();
        double[] sx = new double[n];
        double[] sy = new double[n];
        for (int i = 0; i < n; i++) {
            sx[i] = points.get(i).x;
            sy[i] = points.get(i).y;
        }

        for (int i = 0; i < n - 1; i++) {
            if (sx[i] == sx[i + 1]) {
                throw new IllegalStateException("the control points have duplicate X values");
            }
        }

        double[] d = new double[n - 1];
        for (int i = 0; i < n - 1; i++) {
            double h = sx[i + 1] - sx[i];
            d[i] = (sy[i + 1] - sy[i]) / h;
        }

        double[] sm = new double[n];
        sm[0] = d[0];
        for (int i = 1; i < n - 1; i++) {
            sm[i] = (d[i - 1] + d[i]) * 0.5;
        }
        sm[n - 1] = d[n - 2];

        for (int i = 0; i < n - 1; i++) {
            if (d[i] == 0.0) {
                sm[i] = 0.0;
                sm[i + 1] = 0.0;
            } else {
                double a = sm[i] / d[i];
                double b = sm[i + 1] / d[i];
                double h = Math.hypot(a, b);
                if (h > 9.0) {
                    double t = 3.0 / h;
                    sm[i] = t * a * d[i];
                    sm[i + 1] = t * b * d[i];
                }
            }
        }

        x.clear();
        y.clear();
        m.clear();
        for (int i = 0; i < n; i++) {
            x.add(sx[i]);
            y.add(sy[i]);
            m.add(sm[i]);
        }
        return this;
    }

    public double get(double input) {
        int n = x.size();
        if (n == 0) {
            throw new IllegalStateException("CreateLUT() must be called before get()");
        }

        if (Double.isNaN(input)) {
            return input;
        }

        if (input < x.get(0) || input > x.get(n - 1)) {
            throw new IllegalArgumentException(String.format(
                    "user requested value outside of bounds of LUT. Bounds are: %f to %f. Value provided was: %f",
                    x.get(0), x.get(n - 1), input));
        }

        int i = 0;
        while (i < n - 2 && input >= x.get(i + 1)) {
            i++;
        }

        if (input == x.get(i)) {
            return y.get(i);
        }

        double h = x.get(i + 1) - x.get(i);
        double t = (input - x.get(i)) / h;

        return (y.get(i) * (1 + 2 * t) + h * m.get(i) * t) * (1 - t) * (1 - t)
                + (y.get(i + 1) * (3 - 2 * t) + h * m.get(i + 1) * (t - 1)) * t * t;
    }

    @Override
    public String toString() {
        StringBuilder str = new StringBuilder("[");
        for (int i = 0; i < x.size(); i++) {
            if (i != 0) {
                str.append(", ");
            }
            str.append(String.format("(%s, %s: %s)", x.get(i), y.get(i), m.get(i)));
        }
        str.append("]");
        return str.toString();
    }

    private record Point(double x, double y) {
    }
}
