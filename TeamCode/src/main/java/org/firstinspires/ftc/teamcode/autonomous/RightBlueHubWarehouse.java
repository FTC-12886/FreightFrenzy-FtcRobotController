/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

// import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Manipulator;
import org.firstinspires.ftc.teamcode.ObjDetect;
import org.firstinspires.ftc.teamcode.util.ProfileTrapezoidal;

import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="right blue, hub, park warehouse", group="Blue", preselectTeleOp = "Enhanced TeleOp")

public class RightBlueHubWarehouse extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rearRightDrive = null;
    private DcMotorEx rearLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx armLift;
    private DcMotor clawLeft;
    private DcMotor clawRight;
    //    private DigitalChannel armLimit;
    private State autonomousState = State.EXIT_START;
    private DistanceSensor rearDistance;
    private BNO055IMU imu;

    private DigitalChannel armLimit;
    private boolean lastArmLimitState;
    private boolean armLimitState;

    private double armTargetRaw;
    private Manipulator.ArmPosition lastArmPosition = Manipulator.ArmPosition.UNKNOWN;
    private Manipulator.ArmPosition armPosition = Manipulator.ArmPosition.UNKNOWN;
    private ProfileTrapezoidal trap;
    private ElapsedTime dt = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private ObjDetect objDetect;
    private Manipulator.ArmPosition position;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rearRightDrive  = hardwareMap.get(DcMotorEx.class, "rear_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotorEx.class, "rear_left_drive");
        frontRightDrive  = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");

        armLift = hardwareMap.get(DcMotorEx.class, "arm_lift");
        clawLeft = hardwareMap.get(DcMotorEx.class, "claw_left");
        clawRight = hardwareMap.get(DcMotorEx.class, "claw_right");

        armLimit = hardwareMap.get(DigitalChannel.class, "arm_limit");
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        clawRight.setDirection(DcMotor.Direction.REVERSE);
        clawLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearDistance = hardwareMap.get(DistanceSensor.class, "rear_distance");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set PID gains
        armLift.setVelocityPIDFCoefficients(20, 3, 2,0); // stability limit is p = 40; reduce p or apply damping
        PIDFCoefficients velocityGains = armLift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("velocity", "p (%.2f), i (%.2f), d (%.2f), f (%.2f)", velocityGains.p, velocityGains.i, velocityGains.d, velocityGains.f) ;
        armLift.setPositionPIDFCoefficients(10);
        PIDFCoefficients positionGains = armLift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("position", "p (%.2f), i (%.2f), d (%.2f), f (%.2f)", positionGains.p, positionGains.i, positionGains.d, positionGains.f) ;

        initGyro();
        initObjDetect();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        switch (objDetect.getPosition()) {
            case 1:
                position = Manipulator.ArmPosition.BOTTOM;
                break;
            case 2:
                position = Manipulator.ArmPosition.MIDDLE;
                break;
            case 3:
                position = Manipulator.ArmPosition.TOP;
                break;
            case 0:
            case -1:
                position = Manipulator.ArmPosition.UNKNOWN;
                break;
        }
        telemetry.addData("position", position);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        if (position == Manipulator.ArmPosition.UNKNOWN) {
            armTargetRaw = Manipulator.ArmPosition.TOP.encoderTicks;
        } else {
            armTargetRaw = position.getEncoderTicks();
        }
        runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int armEncoder = armLift.getCurrentPosition();
        armLimitState = armLimit.getState();
        double leftPower = 0;
        double rightPower = 0;
        double clawLeftPower = 0;
        double clawRightPower = 0;
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        double rearCm = rearDistance.getDistance(DistanceUnit.CM);


        telemetry.addData("DISTANCE", rearCm);
        telemetry.addData("STATE", autonomousState);
        telemetry.addData("ANGLE", angle);
        switch (autonomousState) {
            case EXIT_START:
                leftPower = 1;
                rightPower = 1;
                if (rearCm > 35) {
                    leftPower = 0;
                    rightPower = 0;
                    autonomousState = State.TURN_HUB;
                }
                break;
            case TURN_HUB:
                leftPower = 0.00;
                rightPower = 0.75;
                if (angle >= 45) {
                    leftPower = 0;
                    rightPower = 0;
                    runtime.reset();
                    autonomousState = State.DRIVE_FORWARDS;
                }
                break;
            case DRIVE_FORWARDS:
                leftPower = 0.75;
                rightPower = 0.75;
                if (runtime.milliseconds() >= 450) {
                    runtime.reset();
                    autonomousState = State.DROP_BLOCK;
                }
                break;
            case DROP_BLOCK:
                clawLeftPower = 1;
                clawRightPower = 1;
                if (runtime.milliseconds() > 750) {
                    clawLeftPower = 0;
                    clawRightPower = 0;
                    runtime.reset();
                    autonomousState = State.DRIVE_REVERSE;
                }
                break;
            case DRIVE_REVERSE:
                leftPower = -1;
                rightPower = -1;
                if (runtime.milliseconds() >= 500) {
                    leftPower = 0;
                    rightPower = 0;
                    runtime.reset();
                    autonomousState = State.TURN_WAREHOUSE;
                }
                break;
            case TURN_WAREHOUSE:
                leftPower = 0.00;
                rightPower = -0.75;
                if (angle <= -85) {
                    leftPower = 0;
                    rightPower = 0;
                    armPosition = Manipulator.ArmPosition.BOTTOM;
                    armTargetRaw = armPosition.encoderTicks;
                    runtime.reset();
                    autonomousState = State.DRIVE_WAREHOUSE;
                }
                break;
            case DRIVE_WAREHOUSE:
                leftPower = -1;
                rightPower = -1;
                if (rearCm <= 30 || runtime.seconds() > 10) {
                    leftPower = 0;
                    rightPower = 0;
                    autonomousState = State.END;
                }
                break;
            default:
                leftPower = 0;
                rightPower = 0;
                break;

        }
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftPower *= 0.6;
        rightPower *= 0.6;
        rearLeftDrive.setPower(leftPower);
        rearRightDrive.setPower(rightPower);
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);


        clawLeft.setPower(clawLeftPower);
        clawRight.setPower(clawRightPower);
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

        lastArmLimitState = armLimitState;


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    private void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private void initObjDetect() {
        String tfodModelFile = AppUtil.FIRST_FOLDER + "/tflitemodels/TeamElement.tflite";
        String[] labels = {
                "Team"
        };

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "ASg2QBr/////AAABmU3Gzjd/akXrk1NzMQrLNgN6wxZIJ3H7AHf8eU6cL+4hcspa6m1glKBeuuSXaELDtK5J81Ewk7+bYxWFk66Y8qupXK8Hqo81er+2T7R7gfZ5O+dCnJpBmU394oA0PrT2L1qAn3ArLA9bkjNM7xauWiff4YtcuSyDBbBGcMJz1BUDMSJ5az94/XlX+d3ATUBiR3T82RSPXZfv6dn+TvIDr1DqLNwgQnzgTPWZwgITgvAAscBjxETX4CgzThrOShqVkKxAtWOyj+uuU53UIhNHsVMEsJuafMqg+Mhkp6c/+VP6LoFPDJwGwdMxrFByCf2GAKkxmWFTQzreHtwVsN2u5O8wXOGlL5WCi3L1R5Iw7MaW";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(tfodModelFile, labels);

        ObjDetect.ProcessingParameters processingParameters = new ObjDetect.ProcessingParameters();
        this.objDetect = new ObjDetect(tfod, processingParameters);
    }

    public enum State {
        EXIT_START,
        TURN_HUB,
        DRIVE_FORWARDS,
        DROP_BLOCK,
        TURN_WAREHOUSE,
        DRIVE_WAREHOUSE,
        DRIVE_REVERSE,
        END
    }
}
