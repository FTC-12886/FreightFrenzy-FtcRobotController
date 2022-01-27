/*
 * Jay Jasper smooth_delay.c
 * Ported to Java by Caleb Li
 */
package org.firstinspires.ftc.teamcode.util;

public class SmoothDelayImpl extends SmoothDelay {
    private final SmoothDelayInfo profile;

    public SmoothDelayImpl(int steps) {
        this.profile = profileSmoothDelaySetup(steps);
    }

    private SmoothDelayInfo profileSmoothDelaySetup(int steps) {
        SmoothDelayInfo profile = new SmoothDelayInfo();
        profile.stepCurrent = 0;
        profile.steps = steps;

        // init coefficients
        profile.coefficients = new double[steps];
        double sum = 0;

        // fill coefficients
        for (int i = 0; i < profile.steps; ++i) {
            double t = i;
            profile.coefficients[i] = -(t + 1) * (t - profile.steps);
            sum += profile.coefficients[i];
        }

        // normalize coefficients
        for (int i = 0; i < profile.steps; ++i) {
            profile.coefficients[i] /= sum;
        }

        // initialize the input history array
        profile.history = new double[steps];
        for (int i = 0; i < profile.steps; ++i) {
            profile.history[i] = 0;
        }

        return profile;
    }

    // updates history with the new input, spits out a filtered value
    // CAUTION - relies on calling loop to operate at a constant frequency (can we do that??)

    @Override
    public double profileSmoothDelaySmooth(double curentRaw) {
        profile.history[profile.stepCurrent] = curentRaw;

        // discrete convolution integral;
        double result = 0;
        for (int historyIndex = 0; historyIndex < profile.steps; ++historyIndex) {
            int profileIndex = historyIndex - profile.stepCurrent - 1;

            if (profileIndex >= profile.steps) {
                profileIndex -= profile.steps;
            }
            if (profileIndex < 0) {
                profileIndex += profile.steps;
            }

            // actually compute the integral
            result += profile.coefficients[profileIndex] * profile.history[historyIndex];
        }
        profile.stepCurrent = (profile.stepCurrent + 1) % profile.steps;
        return result;
    }

    @Override
    public void profileSmoothDelayFree() {
        // no need to free anything, Java handles memory allocation on its own
    }

    @Override
    public void profileSmoothDelayPrintCoefficients() {
        System.out.printf("Profile Coefficients: %d\n", profile.steps);
        for (int i = 0; i < profile.steps; ++i) {
            System.out.printf("%s\t", profile.coefficients[i]);
        }
        System.out.print("\n");
    }
}
