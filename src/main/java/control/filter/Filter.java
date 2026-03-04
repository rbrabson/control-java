package control.filter;

public interface Filter {
    double estimate(double measurement);

    void reset();

    double getGain();
}
