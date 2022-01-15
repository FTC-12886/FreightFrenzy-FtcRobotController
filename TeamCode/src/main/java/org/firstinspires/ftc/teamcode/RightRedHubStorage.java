package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.List;

@Autonomous(name="right red, hub, park storage", group="Red", preselectTeleOp = "wroking Teleop")
public class RightRedHubStorage extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor rearRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor armLift;
    private DcMotor clawLeft;
    private DcMotor clawRight;
    //    private DigitalChannel armLimit;
    private State autonomousState = State.EXIT_START;
    private DistanceSensor rearDistance;
    private BNO055IMU imu;

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
        rearRightDrive  = hardwareMap.get(DcMotor.class, "rear_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        frontRightDrive  = hardwareMap.get(DcMotor.class, "front_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");

        armLift = hardwareMap.get(DcMotor.class, "arm_lift");
        clawLeft = hardwareMap.get(DcMotor.class, "claw_left");
        clawRight = hardwareMap.get(DcMotor.class, "claw_right");

//        armLimit = hardwareMap.get(DigitalChannel.class, "arm_limit");
//        armLimit.setMode(DigitalChannel.Mode.INPUT);


        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        clawRight.setDirection(DcMotor.Direction.REVERSE);
        clawLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearDistance = hardwareMap.get(DistanceSensor.class, "rear_distance");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        initGyro();
        initObjDetect();
        // initVuforia();
        // initTfod();
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
            armLift.setTargetPosition(Manipulator.ArmPosition.TOP.encoderTicks);
        } else {
            armLift.setTargetPosition(position.encoderTicks);
        }
        armLift.setPower(0.3);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftPower = 0;
        double rightPower = 0;
        double clawLeftPower = 0;
        double clawRightPower = 0;
        telemetry.addData("Status", "Run Time: " + runtime);

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
                if (runtime.milliseconds() >= 500) {
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
                    autonomousState = State.TURN_STORAGE;
                }
                break;
            case TURN_STORAGE:
                leftPower = 0;
                rightPower = 0.75;
                if (angle >= 75) {
                    leftPower = 0;
                    rightPower = 0;
                    armLift.setTargetPosition(-300);
                    armLift.setPower(0.6);
                    armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    autonomousState = State.DRIVE_STORAGE;
                }
                break;
            case DRIVE_STORAGE:
                leftPower = 1;
                rightPower = 1;
                if (runtime.milliseconds() > 5000) {
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
        rearLeftDrive.setPower(leftPower);
        rearRightDrive.setPower(rightPower);
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        clawLeft.setPower(clawLeftPower);
        clawRight.setPower(clawRightPower);


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
        TURN_STORAGE,
        DRIVE_STORAGE,
        TURN_END,
        DRIVE_REVERSE,
        END
    }
}

