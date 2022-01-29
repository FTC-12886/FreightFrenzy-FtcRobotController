package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.ProfileTrapezoidal;
import org.firstinspires.ftc.teamcode.util.SmoothDelay;

import java.util.List;

@TeleOp(name="Single Motor Test TeleOp")
public class SingleMotorTest extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rearRightDrive = null;
    private DcMotorEx rearLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx frontLeftDrive = null;
    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private static final int MAX_VELOCITY_TPS = 2400;

    private final SmoothDelay rightStickSmoothDelay = new SmoothDelay(10);
    private final SmoothDelay leftStickSmoothDelay = new SmoothDelay(10);
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // get DcMotorEx or get DcMotor and cast to DcMotorEx
        driveMotors[0] = rearRightDrive  = hardwareMap.get(DcMotorEx.class, "rear_right_drive");
        driveMotors[1] = rearLeftDrive = hardwareMap.get(DcMotorEx.class, "rear_left_drive");
        driveMotors[2] = frontRightDrive  = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        driveMotors[3] = frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");

        // TODO tuning! leaving these commented to start since Caleb reports the setpoint *is* being reached; the defaults are ok?
        // need to set both velocity and position coefficients since the position controller just sets velocity goals
        //armLift.setVelocityPIDFCoefficients(1.0, 0.1, 0.1, 0.1) // p, i, d, f
        //armLift.setPositionPIDFCoefficients(1.0) // p

        // max speed - 1800, max acceleration 3600
        // testing - 200, 1000
        // some shuddering on descent, not very smooth; going up is fine

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double fastMode = gamepad1.left_stick_button ? 0.85 : 0.60;
        // POV Mode uses left stick to go forward, and right stick to turn.
        // apply smooth delay
        double leftStickY = leftStickSmoothDelay.profileSmoothDelaySmooth(gamepad1.left_stick_y);
        double rightStickX = rightStickSmoothDelay.profileSmoothDelaySmooth(gamepad1.right_stick_x);
        // calculate drive and turn
        double drive = -leftStickY*Math.abs(leftStickY);
        double turn  =  rightStickX*Math.abs(rightStickX);
        // max speed is 165 rpm according to TetrixMotor.java. velocity is in rpm
        double leftVelocity = Range.scale(Range.clip(drive + turn, -1, 1), -1, 1, -MAX_VELOCITY_TPS, MAX_VELOCITY_TPS);
        double rightVelocity = Range.scale(Range.clip(drive - turn, -1, 1), -1, 1, -MAX_VELOCITY_TPS, MAX_VELOCITY_TPS);
        // Calculate fast mode
        leftVelocity *= fastMode;
        rightVelocity *= fastMode;

        // Send power to motors
        switch (getGamepadButtons(gamepad1)) {
            case 'a':
                rearLeftDrive.setVelocity(leftVelocity);
                frontLeftDrive.setVelocity(0);
                rearRightDrive.setVelocity(0);
                frontRightDrive.setVelocity(0);
                break;
            case 'b':
                rearRightDrive.setVelocity(rightVelocity);
                rearLeftDrive.setVelocity(0);
                frontLeftDrive.setVelocity(0);
                frontRightDrive.setVelocity(0);
                break;
            case 'x':
                frontLeftDrive.setVelocity(leftVelocity);
                rearLeftDrive.setVelocity(0);
                rearRightDrive.setVelocity(0);
                frontRightDrive.setVelocity(0);
                break;
            case 'y':
                frontRightDrive.setVelocity(rightVelocity);
                rearLeftDrive.setVelocity(0);
                frontLeftDrive.setVelocity(0);
                rearRightDrive.setVelocity(0);
                break;
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Desired", "left (%.2f), right (%.2f)", leftVelocity, rightVelocity);
        telemetry.addData("Real", "RL (%.2f), RR (%.2f), FL (%.2f), FR (%.2f)", rearLeftDrive.getVelocity(), rearRightDrive.getVelocity(), frontLeftDrive.getVelocity(), frontRightDrive.getVelocity());
        telemetry.addData("controls", "a = RL, b = RR, x = FL, y = FR");
    }
    private char getGamepadButtons(Gamepad gamepad) {
        if (gamepad.a) {
            return 'a';
        } else if (gamepad.b) {
            return 'b';
        } else if (gamepad.x) {
            return 'x';
        } else if (gamepad.y) {
            return 'y';
        } else if (gamepad.dpad_down) {
            return 'd';
        } else if (gamepad.dpad_up) {
            return 'u';
        } else
            return '\u0000';
    }

}
