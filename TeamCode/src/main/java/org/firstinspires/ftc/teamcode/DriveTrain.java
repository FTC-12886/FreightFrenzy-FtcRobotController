package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTrain {
    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx rearLeftDrive;
    private DcMotorEx rearRightDrive;

    public static final double ANGLE_TOLERANCE = 1;
    public static final double DISTANCE_TOLERANCE = 2.0;
    public static final double ENCODER_TOLERANCE = 15;

    /**
     * Parametrized constructor for drive train, takes in the motors
     *
     * @param frontLeftDrive  front left drive motor
     * @param frontRightDrive front right drive motor
     * @param rearLeftDrive   rear left drive motor
     * @param rearRightDrive  rear right drive motor
     */
    public DriveTrain(DcMotorEx frontLeftDrive, DcMotorEx frontRightDrive, DcMotorEx rearLeftDrive, DcMotorEx rearRightDrive) {
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.rearLeftDrive = rearLeftDrive;
        this.rearRightDrive = rearRightDrive;
    }

    /**
     * Sets directions/modes/zero power behaviors for the motors
     *
     * @return drive train object
     */
    public DriveTrain init() {
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[]{rearRightDrive, rearLeftDrive, frontRightDrive, frontLeftDrive}) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // TODO: RUN_WITHOUT_ENCODER or RUN_USING_ENCODER?
        }
        return this;
    }

    /**
     * Stops the motors and resets the encoders
     * Unclear whether motors enter brake or float mode when stopped
     */
    public void resetEncoders() {
        for (DcMotorEx motor : new DcMotorEx[]{rearRightDrive, rearLeftDrive, frontRightDrive, frontLeftDrive}) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Returns the encoder position of the motors. Order is front left, front right, rear left, rear right
     *
     * @return the encoder position of all the motors
     */
    public double[] getEncoderValues() {
        return new double[]{frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(), rearLeftDrive.getCurrentPosition(), rearRightDrive.getCurrentPosition()};
    }

    /**
     * Drives the robot with the specified power
     *
     * @param frontLeft  power to drive the front left motor at
     * @param frontRight power to drive the front right motor at
     * @param rearLeft   power to drive the rear left motor at
     * @param rearRight  power to drive the rear right motor at
     */
    public void setPower(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        frontLeftDrive.setPower(frontLeft);
        frontRightDrive.setPower(frontRight);
        rearLeftDrive.setPower(rearLeft);
        rearRightDrive.setPower(rearRight);
    }

    /**
     * Drives the robot with the specified power
     *
     * @param power power to drive at
     */
    public void drive(double power) {
        setPower(power, power, power, power);
    }

    /**
     * Brakes the robot
     */
    public void brake() {
        for (DcMotorEx motor : new DcMotorEx[]{rearRightDrive, rearLeftDrive, frontRightDrive, frontLeftDrive}) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        drive(0);
        for (DcMotorEx motor : new DcMotorEx[]{rearRightDrive, rearLeftDrive, frontRightDrive, frontLeftDrive}) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * Adjusts motor power to drive at the given angle
     * Pass the same PIDF controller at every call
     *
     * @param targetAngle The angle to drive at
     * @param angleUnit   The angle unit to use
     * @param pidfController The PIDF controller to use
     * @param power       The power to drive at
     */
    public void driveAngle(double targetAngle, double currentAngle, AngleUnit angleUnit, PIDFController pidfController, double power) {
        // convert the angles to degrees
        double target = angleUnit.toDegrees(targetAngle);
        double current = angleUnit.toDegrees(currentAngle);

        // calculate how far we are from the target angle
        double output = pidfController.calculate(target, current);

        // set the motor powers

        setPower(power * -output, power * output, power * -output, power * output);

        // save the error for the next iteration
    }

    /**
     * Drives the robot until the target distance is reached
     *
     * @param targetDistance  The distance to drive to
     * @param currentDistance The current distance
     * @param distanceUnit    The distance unit the distances are in
     * @param pidfController The PIDF controller to use
     * @param power           The power to drive at
     * @return Whether or not the robot has reached the target distance
     */
    public boolean driveDistance(double targetDistance, double currentDistance, DistanceUnit distanceUnit, PIDFController pidfController, double power) {
        // convert the distances to cm
        double target = distanceUnit.toCm(targetDistance);
        double current = distanceUnit.toCm(currentDistance);

        // calculate how far we are from the target distance

        if (Math.abs(pidfController.getError()) < DISTANCE_TOLERANCE) {
            brake();
            return true;
        }

        // calculate the correction amount
        double output = pidfController.calculate(target, current);

        // set the motor powers
        drive(output);

        // save the error for the next iteration

        return false;
    }

    /**
     * Drives the robot until the target distance is reached at a target angle
     *
     * @param targetDistance  The target distance to drive to
     * @param currentDistance The current distance
     * @param distanceUnit    The distance unit the distances are in
     * @param distancePIDFController The PIDF controller to use for distance
     * @param targetAngle     The target angle to drive at
     * @param currentAngle    The current angle
     * @param angleUnit       The angle unit the angles are in
     * @param anglePIDFController The PIDF controller to use for angle
     * @param power           The power to drive at
     * @return Whether or not the robot has reached the target distance
     */
    public boolean driveDistanceAngle(double targetDistance, double currentDistance, DistanceUnit distanceUnit, PIDFController distancePIDFController,
                                      double targetAngle, double currentAngle, AngleUnit angleUnit, PIDFController anglePIDFController, double power) {
        // convert the distances to cm
        double target = distanceUnit.toCm(targetDistance);
        double current = distanceUnit.toCm(currentDistance);

        // calculate how far we are from the target distance
        if (Math.abs(distancePIDFController.getError()) < DISTANCE_TOLERANCE) {
            brake();
            return true;
        }

        // calculate the correction amount
        double correctionPower = distancePIDFController.calculate(target, current);

        // set the motor powers
        driveAngle(targetAngle, currentAngle, angleUnit, anglePIDFController, power);
        return false;
    }

    /**
     * Drives the robot until the target encoder position is reached
     *
     * @param targetPosition   The target encoder position to drive to
     * @param currentPositions The current encoder positions. Order: front left, front right, back left, back right
     * @param pidfController   The PIDF controller to use
     * @param power            The power to drive at
     * @return Whether or not the robot has reached the target encoder position
     */
    public boolean driveEncoder(double targetPosition, double[] currentPositions, PIDFController pidfController, double power) {
        // store current position as the average of the positions
        double currentPosition = 0;
        for (double position : currentPositions) {
            currentPosition += position;
        }
        currentPosition /= currentPositions.length;

        // calculate how far we are from the target position
        double error = targetPosition - currentPosition;

        if (Math.abs(pidfController.getError()) < ENCODER_TOLERANCE) {
            brake();
            return true;
        }
        double output = pidfController.calculate(targetPosition, currentPosition);

        drive(output);

        return false;
    }

    /**
     * Drives the robot until the target encoder position is reached at a target angle
     *
     * @param targetPosition   The target encoder position to drive to
     * @param currentPositions The current encoder positions. Order: front left, front right, back left, back right
     * @param distancePidfController The PIDF controller to use for distance
     * @param targetAngle      The target angle to drive at
     * @param currentAngle     The current angle
     * @param angleUnit        The angle unit the angles are in
     * @param anglePIDFController The PIDF controller to use for angle
     * @param power            The power to drive at
     * @return Whether or not the robot has reached the target encoder position
     */
    public boolean driveEncoderAngle(double targetPosition, double[] currentPositions, PIDFController distancePidfController,
                                     double targetAngle, double currentAngle, AngleUnit angleUnit, PIDFController anglePIDFController, double power) {
        // store current position as the average of the positions
        double currentPosition = 0;
        for (double position : currentPositions) {
            currentPosition += position;
        }
        currentPosition /= currentPositions.length;

        // calculate how far we are from the target position
        double error = targetPosition - currentPosition;

        if (Math.abs(distancePidfController.getError()) < ENCODER_TOLERANCE) {
            brake();
            return true;
        }
        double correctionPower = distancePidfController.calculate(targetPosition, currentPosition);

        driveAngle(targetAngle, currentAngle, angleUnit, anglePIDFController, correctionPower);

        return false;
    }

    /**
     * Turns the robot about the specified axis
     *
     * @param axis  axis to turn about
     * @param power power to turn at; positive is clockwise, negative is counterclockwise
     */
    public void turn(TurningAxis axis, double power) {
        setPower(axis.getDirection()[0] * power,
                axis.getDirection()[1] * power,
                axis.getDirection()[2] * power,
                axis.getDirection()[3] * power);
    }

    /**
     * Turns the robot to the specified angle
     *
     * @param targetAngle  angle to turn to
     * @param currentAngle current angle of the robot
     * @param angleUnit    unit of the angles
     * @param axis         axis to turn about
     * @param pidfController The PIDF controller to use
     * @param power        power to turn at; positive is clockwise, negative is counterclockwise
     * @return true if the robot has reached the target angle
     */
    public boolean turnAngle(double targetAngle, double currentAngle, AngleUnit angleUnit, TurningAxis axis, PIDFController pidfController, double power) {
        // convert the angles to degrees
        double target = angleUnit.toDegrees(targetAngle);
        double current = angleUnit.toDegrees(currentAngle);

        // calculate how far we are from the target angle


        if (Math.abs(pidfController.getError()) < ANGLE_TOLERANCE) {
            brake();
            return true;
        }

        // calculate the turn power
        double turnPower = pidfController.calculate(target, current);

        // set the motor powers
        turn(axis, turnPower);

        // save the error for the next iteration

        return false;
    }

    enum TurningAxis {
        // power ratios are possible, e.g. drive one motor at 2x power of another, use 0.5 and 1.0
        CENTER(new int[]{1, -1, 1, -1}),
        LEFT(new int[]{0, -1, 0, -1}),
        RIGHT(new int[]{1, 0, 1, 0}),
        TOP(new int[]{1, -1, 0, 0}),
        BOTTOM(new int[]{0, 0, 1, -1}),
        REAR_LEFT(new int[]{1, -1, 0, -1}),
        REAR_RIGHT(new int[]{1, -1, 1, 0}),
        FRONT_LEFT(new int[]{0, -1, 1, -1}),
        FRONT_RIGHT(new int[]{1, 0, 1, -1});

        private final int[] direction;

        /**
         * @param direction wheel powers in order FL, FR, RL, RR
         *                  e.g. 1, -1, 1, -1 for rotating around center clockwise
         */
        TurningAxis(int[] direction) {
            this.direction = direction;
        }

        public int[] getDirection() {
            return direction;
        }

    }

}
