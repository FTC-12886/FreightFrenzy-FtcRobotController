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

import com.qualcomm.hardware.lynx.LynxModule;
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

@Autonomous(name="Abstracted Autonomous", group="Iterative Opmode", preselectTeleOp="Abstracted Teleop")

public class FreightFrenzyAutonomous extends OpMode
{

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
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "chassisLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "chassisRight");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearDistance = hardwareMap.get(DistanceSensor.class, "rearDistance");
        manipulator = new Manipulator(
                hardwareMap.get(DcMotor.class, "armLift"),
                hardwareMap.get(DcMotor.class, "clawLeft"),
                hardwareMap.get(DcMotor.class, "clawRight"));
        initGyro();

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
                if (runtime.milliseconds() >= 500) {
                    manipulator.runIntake(0);
                    autonomousState = State.DRIVE_WALL;
                    runtime.reset();
                }
                break;
            case DRIVE_WALL:
                leftPower = -1;
                rightPower = -1;
                if (runtime.milliseconds() >= 800) {
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
