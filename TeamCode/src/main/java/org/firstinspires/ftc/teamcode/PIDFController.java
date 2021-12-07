package org.firstinspires.ftc.teamcode;

public class PIDFController {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double maxOutput;
    private double minOutput;
    private int maxCyclesIntegral;

    private double integral;
    private double lastError;
    private int count;

    public PIDFController(double kP, double kI, double kD, double kF, double maxOutput, double minOutput, int maxCyclesIntegral) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.maxOutput = maxOutput;
        this.minOutput = minOutput;
        this.maxCyclesIntegral = maxCyclesIntegral;
    }

    public double calculate(double setpoint, double input) {
        double error = setpoint - input;
        integral += error;

        double derivative = error - lastError;
        lastError = error;
        double output = kP * error + kI * (integral / count) + kD * derivative + kF * setpoint;

        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < minOutput) {
            output = minOutput;
        }

        count++;
        if (count > maxCyclesIntegral) {
            integral = 0;
            count = 0;
        }
        return output;
    }

    public double getError() {
        return lastError;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public double getMinOutput() {
        return minOutput;
    }

    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput;
    }

    public int getMaxCyclesIntegral() {
        return maxCyclesIntegral;
    }

    public void setMaxCyclesIntegral(int maxCyclesIntegral) {
        this.maxCyclesIntegral = maxCyclesIntegral;
    }
}
