package control.interplut;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;

/**
 * A lookup table (LUT) for interpolation. This class allows you to create a LUT
 * by adding control points (input-output pairs) and then generates the
 * necessary slopes for cubic Hermite interpolation. The get() method can be
 * used to retrieve interpolated values based on the input.
 */
public class InterpLUT {
    public interface Option extends Consumer<InterpLUT> {
    }

    private final List<Double> x = new ArrayList<>();
    private final List<Double> y = new ArrayList<>();
    private final List<Double> m = new ArrayList<>();

    /**
     * Creates a new InterpLUT with the given options. Each option is applied to
     * this InterpLUT instance, allowing you to add control points and configure the
     * LUT before it is created.
     * 
     * @param options A variable number of Option instances that configure this
     *                InterpLUT.
     */
    public InterpLUT(Option... options) {
        for (Option option : options) {
            option.accept(this);
        }
        createLUT();
    }

    /**
     * Returns an Option that adds a control point to the LUT when applied.
     * 
     * @param x The input value for the control point.
     * @param y The output value corresponding to the input for the control point.
     * @return An Option that adds the specified control point to the LUT when
     *         applied.
     */
    public static InterpLUT.Option add(double x, double y) {
        return p -> p.addPoint(x, y);
    }

    /**
     * Adds a control point to the LUT. Control points are pairs of input and output
     * values that define the shape of the LUT. After adding all control points, the
     * createLUT() method must be called to generate the LUT for interpolation.
     *
     * @param input  The input value for the control point.
     * @param output The output value corresponding to the input for the control
     *               point.
     */
    private void addPoint(double input, double output) {
        x.add(input);
        y.add(output);
    }

    /**
     * Generates the LUT for interpolation based on the added control points. This
     * method calculates the necessary slopes for cubic Hermite interpolation and
     * prepares the LUT for use. After calling this method, the get() method can be
     * used to retrieve interpolated values based on the input.
     */
    private void createLUT() {
        if (x.size() != y.size() || x.size() < 2) {
            throw new IllegalStateException(
                    "there must be at least two control points and the arrays must be of equal length");
        }

        List<Point> points = new ArrayList<>(x.size());
        for (int i = 0; i < x.size(); i++) {
            points.add(new Point(x.get(i), y.get(i)));
        }
        points.sort(Comparator.comparingDouble(Point::x));

        int n = points.size();
        double[] sx = new double[n];
        double[] sy = new double[n];
        for (int i = 0; i < n; i++) {
            sx[i] = points.get(i).x();
            sy[i] = points.get(i).y();
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
    }

    /**
     * Retrieves the interpolated output value corresponding to the given input
     * using cubic Hermite interpolation. The input must be within the bounds of the
     * control points added to the LUT.
     *
     * @param input The input value for which to retrieve the interpolated output.
     *              This value must be within the bounds of the control points added
     *              to the LUT.
     * @return The interpolated output value corresponding to the given input. If
     *         the input is outside the bounds of the control points, an
     *         IllegalArgumentException is thrown. If the input is NaN, NaN is
     *         returned.
     */
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

    /**
     * Returns a string representation of the InterpLUT, showing the control points
     * and their corresponding slopes. The format is a list of tuples, where each
     * tuple contains the input value, output value, and slope for each control
     * point. This can be useful for debugging and visualizing the LUT after it has
     * been created.
     *
     * @return A string representation of the InterpLUT, showing the control points
     *         and their corresponding slopes in a readable format. Each control
     *         point is represented as a tuple of the input value, output value, and
     *         slope, and the entire LUT is represented as a list of these tuples.
     */
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
