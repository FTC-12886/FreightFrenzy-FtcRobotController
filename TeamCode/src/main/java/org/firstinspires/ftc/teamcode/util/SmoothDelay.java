/*
 * Jay Jasper smooth_delay.h
 * Ported to Java by Caleb Li
 */
package org.firstinspires.ftc.teamcode.util;

public abstract class SmoothDelay {
    public static class SmoothDelayInfo{

        public int steps;
        public int stepCurrent;
        public double[] coefficients;
        public double[] history;
    }

//    private abstract SmoothDelayInfo profileSmoothDelaySetup(int steps);

    public abstract double profileSmoothDelaySmooth(double curentRaw);

    public abstract void profileSmoothDelayFree();

    public abstract void profileSmoothDelayPrintCoefficients();

}
