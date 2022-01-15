package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.List;

@Autonomous(name="left red, storage, park storage", group="Red", preselectTeleOp = "wroking Teleop")

public class LeftRedStorageStorage extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor rearRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor armLift;
    private DcMotor clawLeft;
    private DcMotor clawRight;
    private DigitalChannel armLimit;
    private State autonomousState = State.EXIT_START;
    private DistanceSensor rearDistance;
    private BNO055IMU imu;

    public static double RIGHT_BOUNDARY = 0.67;
    public static double LEFT_BOUNDARY = 0.33;
    public static double WIDTH_IGNORE = 0.25;
    public static double HEIGHT_IGNORE = 0.5;
    private static final String[] COLORS = new String[]{"blue", "green", "yellow", "purple", "teal"};
    private static final String TFOD_MODEL_FILE = AppUtil.FIRST_FOLDER + "/tflitemodels/TeamElement.tflite";
    private static final String[] LABELS = {
            "Team"
    };
    private static final String VUFORIA_KEY =
            "ASg2QBr/////AAABmU3Gzjd/akXrk1NzMQrLNgN6wxZIJ3H7AHf8eU6cL+4hcspa6m1glKBeuuSXaELDtK5J81Ewk7+bYxWFk66Y8qupXK8Hqo81er+2T7R7gfZ5O+dCnJpBmU394oA0PrT2L1qAn3ArLA9bkjNM7xauWiff4YtcuSyDBbBGcMJz1BUDMSJ5az94/XlX+d3ATUBiR3T82RSPXZfv6dn+TvIDr1DqLNwgQnzgTPWZwgITgvAAscBjxETX4CgzThrOShqVkKxAtWOyj+uuU53UIhNHsVMEsJuafMqg+Mhkp6c/+VP6LoFPDJwGwdMxrFByCf2GAKkxmWFTQzreHtwVsN2u5O8wXOGlL5WCi3L1R5Iw7MaW";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        initGyro();
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

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        armLift.setTargetPosition(-300);
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
                if (rearDistance.getDistance(DistanceUnit.CM) > 35) {
                    leftPower = 0;
                    rightPower = 0;
                    autonomousState = State.TURN_STORAGE;
                }
                break;
            case TURN_STORAGE:
                leftPower = 0;
                rightPower = 0.75;
                if (angle >= 65) {
                    leftPower = 0;
                    rightPower = 0;
                    runtime.reset();
                    autonomousState = State.DRIVE_FORWARDS;
                }
                break;
            case DRIVE_FORWARDS:
                leftPower = 1;
                rightPower = 1;
                if (runtime.milliseconds() >= 250) {
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
            case DRIVE_STORAGE:
                leftPower = 1;
                rightPower = 1;
                armLift.setTargetPosition(-300);
                armLift.setPower(0.6);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (runtime.milliseconds() >= 1000) {
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


    public enum State {
        EXIT_START,
        TURN_90,
        DRIVE_FORWARDS,
        DROP_BLOCK,
        TURN_STORAGE,
        DRIVE_REVERSE,
        DRIVE_STORAGE,
        END
    }
}

