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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ProfileTrapezoidal;
import org.firstinspires.ftc.teamcode.util.SmoothDelay;

import java.util.List;

@TeleOp(name="Enhanced Decoupled TeleOp")
public class EnhancedDecoupledTeleOp extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rearRightDrive = null;
    private DcMotorEx rearLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx frontLeftDrive = null;
    private final DcMotorEx[] driveMotors = new DcMotorEx[4];

    private DcMotorEx armLift;
    private DcMotor clawLeft;
    private DcMotor clawRight;

    private DigitalChannel armLimit;
    private boolean lastArmLimitState;

    private double armTargetRaw;
    private Manipulator.ArmPosition lastArmPosition = Manipulator.ArmPosition.UNKNOWN;
    private Manipulator.ArmPosition armPosition = Manipulator.ArmPosition.UNKNOWN;
    private ProfileTrapezoidal trap;
    private final ElapsedTime dt = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private final SmoothDelay rightStickSmoothDelay = new SmoothDelay(1);
    private final SmoothDelay leftStickSmoothDelay = new SmoothDelay(1);


    // todo simplify any expressions (make sure they are still readable though)
    // CPR taken from manufacturer website
    private static final double ARM_CPR = 2880;
    private static final double ARM_ENCODER_OFFSET = ((Math.PI/2)-(Math.PI/12))/(2*Math.PI)*ARM_CPR; // todo plug in "lost" angle (in place of pi/12) or just measure
    private static final double ARM_LENGTH = 40 * 0.01; // todo calculate arm length to claw
    private static final double WHEEL_DIAMETER = DistanceUnit.INCH.toMeters(4); // taken from manufacturer website
    private static final double WHEEL_CPR = 480;
    private static final int MAX_VELOCITY_TPS = 2400;
    private static final double MAX_VELOCITY_MPS = ticksToMeters(MAX_VELOCITY_TPS);

    /* TODO calculate arm angle with encoder, input things
     * 1440 cpr with 60:1 gearbox
     * 2880 cpr with 2:1 external gear ratio (40 teeth: 80 teeth)
     * Use trig to determining angle "lost" -- see picture in notes
     */
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

        armLift = hardwareMap.get(DcMotorEx.class, "arm_lift");
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLimit = hardwareMap.get(DigitalChannel.class, "arm_limit");
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        // TODO tuning! leaving these commented to start since Caleb reports the setpoint *is* being reached; the defaults are ok?
        // need to set both velocity and position coefficients since the position controller just sets velocity goals
        //armLift.setVelocityPIDFCoefficients(1.0, 0.1, 0.1, 0.1) // p, i, d, f
        //armLift.setPositionPIDFCoefficients(1.0) // p

        // max speed - 1800, max acceleration 3600
        // testing - 200, 1000
        // some shuddering on descent, not very smooth; going up is fine
        trap = new ProfileTrapezoidal(2000, 4000); // cruise speed, acceleration TODO adjust these
        dt.reset();

        clawLeft = hardwareMap.get(DcMotor.class, "claw_left");
        clawRight = hardwareMap.get(DcMotor.class, "claw_right");
        clawLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRight.setDirection(DcMotor.Direction.REVERSE);

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

        rearRightDrive.setVelocityPIDFCoefficients(10, 3, 0, 0.00029574);
        rearLeftDrive.setVelocityPIDFCoefficients(10, 3, 0, 0.00029574);
        frontRightDrive.setVelocityPIDFCoefficients(10, 3, 0, 0.00029574);
        frontLeftDrive.setVelocityPIDFCoefficients(10, 3, 0, 0.00029574);

        // set PID gains
        armLift.setVelocityPIDFCoefficients(20, 3, 2,0); // stability limit is p = 40; reduce p or apply damping
        PIDFCoefficients velocityGains = armLift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("velocity", "p (%.2f), i (%.2f), d (%.2f), f (%.2f)", velocityGains.p, velocityGains.i, velocityGains.d, velocityGains.f) ;
        armLift.setPositionPIDFCoefficients(10);
        PIDFCoefficients positionGains = armLift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("position", "p (%.2f), i (%.2f), d (%.2f), f (%.2f)", positionGains.p, positionGains.i, positionGains.d, positionGains.f) ;

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
        int armEncoder = armLift.getCurrentPosition();
        boolean armLimitState = armLimit.getState();
        telemetry.addData("arm pos", armEncoder);

        // both triggers = duck mode
        if (gamepad1.right_trigger > 0.00 && gamepad1.left_trigger > 0.00) {
            clawLeft.setPower(gamepad1.left_trigger);
            clawRight.setPower(-gamepad1.right_trigger);
            armPosition = Manipulator.ArmPosition.DUCK; // handle later

        } else if (gamepad1.right_trigger > 0.0){ // right is out, positive is out
            clawLeft.setPower(gamepad1.right_trigger);
            clawRight.setPower(gamepad1.right_trigger);

        } else if (gamepad1.left_trigger > 0.0) { // left is in, negative is in
            clawLeft.setPower(-gamepad1.left_trigger);
            clawRight.setPower(-gamepad1.left_trigger);

        } else { // halt motors if no buttons are pressed
            clawLeft.setPower(0);
            clawRight.setPower(0);
        }

        char button = getGamepadButtons(gamepad1);
        telemetry.addData("button", button);
        switch (button) {
            case 'a':
                armPosition = Manipulator.ArmPosition.GROUND;
                armTargetRaw = armPosition.encoderTicks;
                break;
            case 'b':
                armPosition = Manipulator.ArmPosition.BOTTOM;
                armTargetRaw = armPosition.encoderTicks;
                break;
            case 'y':
                armPosition = Manipulator.ArmPosition.MIDDLE_TELEOP;
                armTargetRaw = armPosition.encoderTicks;
                break;
            case 'x':
                armPosition = Manipulator.ArmPosition.TOP;
                armTargetRaw = armPosition.encoderTicks;
                break;
            case 'd':
                armTargetRaw += 10;
                break;
            case 'u':
                armTargetRaw -= 10;
                break;
        }


        if (armPosition == Manipulator.ArmPosition.DUCK && lastArmPosition != Manipulator.ArmPosition.DUCK) { // transition to duck mode
            armTargetRaw = armPosition.encoderTicks;
        }

        // profile and move arm motor
        double armTargetSmooth = trap.smooth(armTargetRaw, dt.time()/1000.0);
        telemetry.addData("armTargetRaw", (int) armTargetRaw);
        telemetry.addData("armTargetSmooth", armTargetSmooth);
        telemetry.addData("arm state", armPosition);
        telemetry.addData("dt", (int) dt.time());
        dt.reset();

        // don't send commands to motor if we are resetting encoder
        if (armLimitState && !lastArmLimitState) { // when switch not on and last state is on
            armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            armLift.setTargetPosition((int) armTargetSmooth);
            armLift.setPower(1);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double theta = ticksToTheta(trap.getCurrent_position());
        double thetaDot = ticksToThetaDot(trap.getCurrent_velocity());

        double fastMode = gamepad1.left_stick_button ? 0.85 : 0.60;
        if (fastMode > 0.60) { // move arm up if fast mode is on
            armPosition = Manipulator.ArmPosition.BOTTOM;
        }

        // POV Mode uses left stick to go forward, and right stick to turn.
        // apply smooth delay
        double leftStickY = leftStickSmoothDelay.profileSmoothDelaySmooth(gamepad1.left_stick_y);
        double rightStickX = rightStickSmoothDelay.profileSmoothDelaySmooth(gamepad1.right_stick_x)*0.85;

        // calculate drive and turn in m/s, adjust for fast mode to get "real velocity"
        double drive = -leftStickY*Math.abs(leftStickY)*MAX_VELOCITY_MPS*fastMode;
        double turn  =  rightStickX*Math.abs(rightStickX)*MAX_VELOCITY_MPS*fastMode;

        // keep arm in place horizontally (cancel out body relative motion); derivative of arm_length*cos(theta), move at rate of thetadot
        drive -= (-ARM_LENGTH*Math.sin(theta)*thetaDot);

        // converting to tic/s; any velocity modification should happen BEFORE this
        drive = metersToTicks(drive);
        turn = metersToTicks(turn);
        double leftVelocity = Range.clip(drive + turn, -MAX_VELOCITY_TPS, MAX_VELOCITY_TPS);
        double rightVelocity = Range.clip(drive - turn, -MAX_VELOCITY_TPS, MAX_VELOCITY_TPS);
        // Send power to motors
        rearLeftDrive.setVelocity(leftVelocity);
        frontLeftDrive.setVelocity(leftVelocity);
        rearRightDrive.setVelocity(rightVelocity);
        frontRightDrive.setVelocity(rightVelocity);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Desired", "left (%.2f), right (%.2f)", leftVelocity, rightVelocity);
        telemetry.addData("Real", "RL (%.2f), RR (%.2f), FL (%.2f), FR (%.2f)", rearLeftDrive.getVelocity(), rearRightDrive.getVelocity(), frontLeftDrive.getVelocity(), frontRightDrive.getVelocity());

        lastArmPosition = armPosition;
        lastArmLimitState = armLimitState;
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

    private static double thetaToTicks(double theta) {
        return thetaToTicks(theta, AngleUnit.RADIANS);
    }
    private static double thetaToTicks(double theta, AngleUnit angleUnit) {
        theta = angleUnit.toRadians(theta);
        return ARM_ENCODER_OFFSET +theta/(Math.PI*2)*ARM_CPR;
    }

    private static double ticksToTheta(double ticks) {
        return ticksToTheta(ticks, AngleUnit.RADIANS);
    }

    private static double ticksToTheta(double ticks, AngleUnit angleUnit) {
        double theta = (ticks - ARM_ENCODER_OFFSET) * ((2 * Math.PI) / ARM_CPR);
        return angleUnit.fromRadians(theta);
    }

    private static double ticksToThetaDot(double ticks) {
        return ticksToThetaDot(ticks, AngleUnit.RADIANS);
    }

    private static double ticksToThetaDot(double ticks, AngleUnit angleUnit) {
        double thetaDot = ((2 * Math.PI) / ARM_CPR) * ticks; // rad per tick * ticks per second
        return angleUnit.fromRadians(thetaDot);
    }

    private static double ticksToMeters(double ticks) {
        return ticksToDistance(ticks, DistanceUnit.METER);
    }

    private static double ticksToDistance(double ticks, DistanceUnit distanceUnit) {
        double distance = (ticks/WHEEL_CPR)*(WHEEL_DIAMETER*Math.PI);
        return distanceUnit.fromMeters(distance);
    }

    private static double metersToTicks(double meters) {
        return distanceToTicks(meters, DistanceUnit.METER);
    }

    private static double distanceToTicks(double distance, DistanceUnit distanceUnit) {
        distance = distanceUnit.toMeters(distance);
        return distance/(WHEEL_DIAMETER*Math.PI)*WHEEL_CPR;
    }
}
