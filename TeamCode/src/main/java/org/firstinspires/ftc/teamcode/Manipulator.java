package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Manipulator {
    private DcMotor armLift;
    private DcMotor clawLeft;
    private DcMotor clawRight;
    private ArmPosition armState = ArmPosition.GROUND;
    public static final double ARM_UP_SPEED = 0.6;
    public static final double ARM_DOWN_SPEED = 0.3;

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

    public int getArmEncoder() {
        return armLift.getCurrentPosition();
    }
    public void runIntake(boolean reverse) {
        runIntake(reverse ? -1: 1);
    }

    public void runIntake(int power) {
        clawLeft.setPower(-power);
        clawRight.setPower(-power);
    }

    public void runDuckMode(boolean reverse) {
        runDuckMode(reverse ? -1: 1);
    }
    public void runDuckMode(int power) {
        moveArmToPosition(ArmPosition.DUCK);
        clawLeft.setPower(power);
        clawRight.setPower(-power);
    }

    public ArmPosition getArmState() {
        return armState;
    }

    public void moveArmToPosition(ArmPosition armPosition) {
        boolean up = armPosition.getEncoderTicks() > armState.getEncoderTicks();
        if (up)
            moveArmToPosition(armPosition, ARM_UP_SPEED);
        else
            moveArmToPosition(armPosition, ARM_DOWN_SPEED);
    }

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


    public enum ArmPosition {
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
