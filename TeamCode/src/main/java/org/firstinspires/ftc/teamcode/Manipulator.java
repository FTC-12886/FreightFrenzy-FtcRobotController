package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Manipulator {
    private DcMotor armLift;
    private DcMotor clawLeft;
    private DcMotor clawRight;
    private ArmPosition armState;

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

    public void moveArmToPosition(ArmPosition armPosition) {
        moveArmToPosition(armPosition, 0.6);
    }
    public void moveArmToPosition(ArmPosition armPosition, double power) {
        armState = armPosition;
        switch (armState) {
            case GROUND:
                armLift.setTargetPosition(-30); // TODO Use limit switch instead
                break;
            case BOTTOM:
                armLift.setTargetPosition(-300);
                break;
            case MIDDLE:
                armLift.setTargetPosition(-650);
                break;
            case TOP:
                armLift.setTargetPosition(-915);
                break;
            case DUCK:
                armLift.setTargetPosition(-750);
                break;
            default:
                armLift.setTargetPosition(armLift.getCurrentPosition());
                break;
        }

        armLift.setPower(power);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public enum ArmPosition {
        GROUND,
        BOTTOM,
        MIDDLE,
        TOP,
        DUCK
    }
}
