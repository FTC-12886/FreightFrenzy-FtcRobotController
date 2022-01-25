package org.firstinspires.ftc.teamcode.oldrobot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DriveTrain {
    private final DcMotorEx leftDrive;
    private final DcMotorEx rightDrive;
    private final DcMotorEx[] driveMotors;
    private double scaleFactor;

    /**
     * Parametrized constructor
     * @param leftDrive left drive motor
     * @param rightDrive right drive motor
     * @param scaleFactor initial scale factor (multiplied by power before sending to motor)
     * @param hardwareMap hardware map to enable bulk caching
     */
    public DriveTrain(DcMotorEx leftDrive, DcMotorEx rightDrive, int scaleFactor, HardwareMap hardwareMap) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        driveMotors = new DcMotorEx[]{leftDrive, rightDrive};
        this.scaleFactor = scaleFactor;
        init(hardwareMap);
    }

    protected DriveTrain init(HardwareMap hardwareMap) {
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        return this;
    }

    /**
     * Returns the left drive motor power (from -1 to 1)
     * @return the left drive motor power
     */
    public double getLeftDrivePower() {
        return leftDrive.getPower();
    }

    /**
     * Returns the right drive motor power (from -1 to 1)
     * @return the right drive motor power
     */
    public double getRightDrivePower() {
        return rightDrive.getPower();
    }

    /**
     * Sets the power of the motors and scales it according to the scale factor
     * @param leftPower power to send to the left motor
     * @param rightPower power to send to the right motor
     * @return this drive train object
     */
    public DriveTrain setPower(double leftPower, double rightPower) {
        leftDrive.setPower(Range.clip(leftPower * scaleFactor, -1, 1));
        rightDrive.setPower(Range.clip(rightPower * scaleFactor, -1, 1));
        leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        return this;
    }

    /**
     * Sets the scale factor by which to scalae motor powers
     * @param scaleFactor new scale factor
     * @return this drive train object
     */
    public DriveTrain setScaleFactor(double scaleFactor) {
        this.scaleFactor = scaleFactor;
        return this;
    }

    /**
     * Returns the current scale factor
     * @return the current scale factor
     */
    public double getScaleFactor() {
        return scaleFactor;
    }

    /**
     * Sets the zero power behavior of all motors
     * @param zeroPowerBehavior the new zero power behavior
     * @return this drive train object
     */
    public DriveTrain setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        }
        return this;
    }

//    public DcMotorEx getLeftDrive() {
//        return leftDrive;
//    }
//
//    public DcMotorEx getRightDrive() {
//        return rightDrive;
//    }

    /**
     * Return the position of the left drive encoder
     * @return the position of the left drive encoder
     */
    public int getLeftDriveEncoder() {
        return leftDrive.getCurrentPosition();
    }

    /**
     * Return the position of the right drive encoder
     * @return the position of the right drive encoder
     */
    public int getRightDriveEncoder() {
        return leftDrive.getCurrentPosition();
    }

    /**
     * Drives to the encoder positions specified
     * @param leftEncoder the position of the left encoder
     * @param rightEncoder the position of the right encoder
     * @param power the power to mvoe at
     * @return this drive train object
     */
    public DriveTrain driveToEncoder(int leftEncoder, int rightEncoder, double power) {
        leftDrive.setTargetPosition(leftEncoder);
        rightDrive.setTargetPosition(rightEncoder);
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }

        setPower(power, power);
        return this;
    }

    /**
     * Returns whether the left motor is busy
     * @return whether the left motor is busy
     */
    public boolean leftDriveBusy() {
        return leftDrive.isBusy();
    }

    /**
     * Returns whether the right motor is busy
     * @return whether the right motor is busy
     */
    public boolean rightDriveBusy(){
        return rightDrive.isBusy();
    }

    /**
     * Returns true if any motor(s) are busy
     * @return whether one or more motor is busy
     */
    public boolean motorsBusy() {
        return leftDriveBusy() || rightDriveBusy();
    }
    /**
     * Resets the encoders
     * @return this drive train object
     */
    public DriveTrain resetEncoders() {
        // check whether motors are busy first?
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        return this;
    }
}
