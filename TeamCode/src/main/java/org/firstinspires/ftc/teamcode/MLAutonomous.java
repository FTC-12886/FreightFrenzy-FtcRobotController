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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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

@Autonomous(name="ML Autonomous", group="Iterative Opmode", preselectTeleOp="Basic Sample")

public class MLAutonomous extends OpMode
{
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ASg2QBr/////AAABmU3Gzjd/akXrk1NzMQrLNgN6wxZIJ3H7AHf8eU6cL+4hcspa6m1glKBeuuSXaELDtK5J81Ewk7+bYxWFk66Y8qupXK8Hqo81er+2T7R7gfZ5O+dCnJpBmU394oA0PrT2L1qAn3ArLA9bkjNM7xauWiff4YtcuSyDBbBGcMJz1BUDMSJ5az94/XlX+d3ATUBiR3T82RSPXZfv6dn+TvIDr1DqLNwgQnzgTPWZwgITgvAAscBjxETX4CgzThrOShqVkKxAtWOyj+uuU53UIhNHsVMEsJuafMqg+Mhkp6c/+VP6LoFPDJwGwdMxrFByCf2GAKkxmWFTQzreHtwVsN2u5O8wXOGlL5WCi3L1R5Iw7MaW";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Manipulator manipulator;
    private State autonomousState = State.EXIT_START;
    private DistanceSensor rearDistance;
    private BNO055IMU imu;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "chassisLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "chassisRight");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        manipulator = new Manipulator(
                hardwareMap.get(DcMotor.class, "armLift"),
                hardwareMap.get(DcMotor.class, "clawLeft"),
                hardwareMap.get(DcMotor.class, "clawRight"));
        initGyro();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // if (tfod != null) {
        //     double left = 0;
        //     double top = 0;
        //     double right = 0;
        //     double bottom = 0;
        //     // getUpdatedRecognitions() will return null if no new information is available since
        //     // the last time that call was made.
        //     List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        //     if (updatedRecognitions != null) {
        //         telemetry.addData("# Object Detected", updatedRecognitions.size());
        //         // step through the list of recognitions and display boundary info.
        //         int i = 0;
        //         for (Recognition recognition : updatedRecognitions) {
        //             telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
        //             telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
        //                     recognition.getLeft(), recognition.getTop());
        //             telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
        //                     recognition.getRight(), recognition.getBottom());
        //             i++;
        //             if (recognition.getLabel().equals("Duck")) {
        //                 left = recognition.getLeft();
        //                 top = recognition.getTop();
        //                 right = recognition.getRight();
        //                 bottom = recognition.getBottom();
        //             }
        //         }
        //         if (left != 0 && right != 0) {
        //             if (left < 0.33 && right < 0.33) {
        //                 telemetry.addData("Level", "left/level 1");
        //             } else if (left < 0.66 && right < 0.66) {
        //                 telemetry.addData("Level", "left/level 2");
        //             } else if (left < 1 && right < 1) {
        //                 telemetry.addData("Level", "left/level 3");

        //             }
        //         }
        //         telemetry.update();
        //     }
        // } else {
        //     telemetry.addData(">","TFOD is null");
        //     telemetry.update();
        // }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        double leftPower = 0;
        double rightPower = 0;
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("arm pos", manipulator.getArmEncoder());

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        double rearCm = rearDistance.getDistance(DistanceUnit.CM);

        // duck - 10 pt
        // shipping hub - 6 pt
        // duck bonus - 10 pt
        // team element bonus - 20 pt
        // shipping hub is better if can detect barcode
        telemetry.addData("DISTANCE", rearCm);
        telemetry.addData("STATE", autonomousState);
        telemetry.addData("ANGLE", angle);
        switch (autonomousState){
            case EXIT_START:
                manipulator.moveArmToPosition(Manipulator.ArmPosition.TOP);
                leftPower = 1;
                rightPower = 1;
                if (rearCm >= 100) { // 121.92 = 4 ft
                    leftPower = 0;
                    rightPower = 0;
                    autonomousState = State.DRIVE_SHIPPING_HUB;
                }


                break;
            case DRIVE_SHIPPING_HUB:
                rightPower = 1;
                if (angle > 90) {
                    leftPower = 0;
                    rightPower = 0;
                    autonomousState = State.DROP_FREIGHT;
                    runtime.reset();

                }

                break;
            case DROP_FREIGHT:
                manipulator.runIntake(true);
                if (runtime.seconds() >= 3)
                    autonomousState = State.DRIVE_WALL;
                break;
            case DRIVE_WALL:
                leftPower = -1;
                rightPower = -1;
                if (rearCm <= 20) {
                    leftPower = 0;
                    rightPower = 0;
                    autonomousState = State.TURN_BARRIER;
                }

                break;
            case TURN_BARRIER:
                rightPower = 1;
                if (angle > 120) {
                    leftPower = 0;
                    rightPower = 0;
                    autonomousState = State.DRIVE_PARKING;
                    runtime.reset();
                }


                break;
            case DRIVE_PARKING:
                leftPower = 1;
                rightPower = 1;
                if (runtime.seconds() > 5) {
                    autonomousState = State.END;
                }
                break;
            case END:
                leftPower = 0;
                rightPower = 0;
                break;

        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);



        // switch (getGamepadButtons(gamepad1)
        // level 1 = -230
        // level 2 = -480
        // level 3 = -780
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
    /**
     * Initialize the Vuforia localization engine.
     */

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        //tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    public enum State {
        DETECT_BARCODE,
        EXIT_START,
        DRIVE_DUCK,
        DRIVE_SHIPPING_HUB,
        DROP_FREIGHT,
        TURN_BARRIER,
        DRIVE_WALL,
        DRIVE_PARKING,
        END
    }
}
