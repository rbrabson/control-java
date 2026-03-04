package control.examples.interplut.temperature;

import control.interplut.InterpLUT;

public class Main {
    public static void main(String[] args) {
        InterpLUT compensation = new InterpLUT().add(0, 1.0).add(20, 0.95).add(40, 0.90).add(60, 0.82).createLUT();

        double ambient = 35.0;
        double gainScale = compensation.get(ambient);
        System.out.printf("Temperature %.1fC => gain scale %.3f%n", ambient, gainScale);
    }
}
