package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Manipulator {

    private DcMotor armLift;
    private DcMotor clawLeft;
    private DcMotor clawRight;
    private ArmPosition armState = ArmPosition.GROUND;
    public static final double ARM_UP_SPEED = 0.6;
    public static final double ARM_DOWN_SPEED = 0.3;
    private static final double DEFAULT_SPEED = 0.3;

    /**
     * Makes a new Manipulator
     * Assumes arm is in the ground position
     * @param armLift the arm motor
     * @param clawLeft  the left claw motor
     * @param clawRight the right claw motor
     */
    public Manipulator(DcMotor armLift, DcMotor clawLeft, DcMotor clawRight) {
        this.armLift = armLift;
        this.clawLeft = clawLeft;
        this.clawRight = clawRight;
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        clawRight.setDirection(DcMotor.Direction.REVERSE);
        clawLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Gets the current arm position with getCurrentPosition()
     * May return stale values if bulk caching is manual.
     * May add more loop time if not first call in loop.
     * @return current arm position
     */
    public int getArmEncoder() {
        return armLift.getCurrentPosition();
    }

    public void setClawLeftPower(int power) {
        clawLeft.setPower(power);
    }

    public void setClawRightPower(int power) {
        clawRight.setPower(power);
    }
    /**
     * Runs the intake at full power
     * @param reverse true if the intake should be reversed (turn into an out-take)
     */
    public void runIntake(boolean reverse) {
        runIntake(reverse ? -1: 1);
    }

    /**
     * Runs the intake at specified power
     * @param power power to run at
     */
    public void runIntake(int power) {
        setClawLeftPower(-power);
        setClawRightPower(-power);
    }

    /**
     * Runs duck mode at full power
     * @param reverse true if the wheels should be reversed
     */
    public void runDuckMode(boolean reverse) {
        runDuckMode(reverse ? -1: 1);
    }

    /**
     * Runs duck mode at specified power
     * @param power power to run at
     */
    public void runDuckMode(int power) {
        moveArmToPosition(ArmPosition.DUCK);
        setClawLeftPower(power);
        setClawRightPower(-power);
    }

    /**
     * Gets the current arm state
     * @return the current arm state
     */
    public ArmPosition getArmState() {
        return armState;
    }

    /**
     * Moves the arm to the given position
     * @param armPosition the position to move to
     */
    public void moveArmToPosition(ArmPosition armPosition) {
        if (armState == ArmPosition.UNKNOWN) {
            moveArmToPosition(armPosition, DEFAULT_SPEED);
        } else {
            boolean up = armPosition.getEncoderTicks() > armState.getEncoderTicks();
            if (up)
                moveArmToPosition(armPosition, ARM_UP_SPEED);
            else
                moveArmToPosition(armPosition, ARM_DOWN_SPEED);
        }
    }

    /**
     * Moves the arm to specified position.
     * @param armPosition position to move to
     * @param power power to move at
     */
    public void moveArmToPosition(ArmPosition armPosition, double power) {
        armState = armPosition;

        armLift.setTargetPosition(armState.getEncoderTicks());
        armLift.setPower(power); // should we use set power or set velocity?
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (armState == ArmPosition.GROUND)
            new Thread(new Runnable() {
                @Override
                public void run() {
                    while (armLift.isBusy());
                    armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }).start();
    }

    /**
     * Moves the arm to specified encoder position.
     * Arm will run at default speed if next call is moveArmToPosition(ArmPosition)
     * @param encoderTicks position to move to
     * @param power power to move at
     */
    public void moveArmToPosition(int encoderTicks, double power) {
        armLift.setTargetPosition(encoderTicks);
        armLift.setPower(power); // should we use set power or set velocity?
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armState = ArmPosition.UNKNOWN;
    }
    public void nudgeArm(int encoderTicks, int currentEncoderTicks) {
        nudgeArm(encoderTicks, currentEncoderTicks, DEFAULT_SPEED);
    }
    public void nudgeArm(int encoderTicks, int currentEncoderTicks, double power) {
        moveArmToPosition(currentEncoderTicks + encoderTicks, power);
    }


    public enum ArmPosition {
        UNKNOWN (0),
        GROUND (-30),
        BOTTOM (-300),
        MIDDLE (-650),
        DUCK (-750),
        TOP (-915);

        private final int encoderTicks;

        ArmPosition(int encoderTicks) {
            this.encoderTicks = encoderTicks;
        }

        public int getEncoderTicks() {
            return encoderTicks;
        }
    }
}
