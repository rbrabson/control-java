package control.examples.interplut.basic;

import control.interplut.InterpLUT;

import static control.interplut.InterpLUT.add;

public class Main {
    public static void main(String[] args) {
        InterpLUT lut = new InterpLUT(add(0, 0), add(1000, 0.2), add(2000, 0.55), add(3000, 1.0));

        double lookup = 1500;
        System.out.printf("LUT(%f) = %.4f%n", lookup, lut.get(lookup));
    }
}
