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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import ProfileTrapezoidal;

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

@TeleOp(name="wroking Teleop", group="Iterative Opmode")

public class WorkingTeleOp extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor rearRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotorEx armLift;
    private DcMotor clawLeft;
    private DcMotor clawRight;
    //    private DigitalChannel armLimit;
    private double armTargetRaw;
    private int STATE_HOLD = 0;
    private int STATE_DUCK_INIT = 1;
    private int STATE_DUCK_HOLD = 2;
    private int armState;
    private ProfileTrapezoidal trap;
    private ElapsedTime dt = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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

        armLift = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_lift");
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // TODO remove here.. and replace with zero-ing procedure
        armLift.setDirection(DcMotor.Direction.FORWARD);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // only have to do once, in setup

        // TODO tuning! leaving these commented to start since Caleb reports the setpoint *is* being reached; the defaults are ok?
        // need to set both velocity and position coefficients since the position controller just sets velocity goals 
        //armLift.setVelocityPIDFCoefficients(1.0, 0.1, 0.1, 0.1) // p, i, d, f
        //armLift.setPositionPIDFCoefficients(1.0) // p

        armTargetRaw = -300; 
        armState = STATE_HOLD;
        trap = new ProfileTrapezoidal(2000, 4000); // cruise speed, acceleration // TODO adjust these
        dt.reset();

//        armLimit = hardwareMap.get(DigitalChannel.class, "arm_limit");
//        armLimit.setMode(DigitalChannel.Mode.INPUT);

        clawLeft = hardwareMap.get(DcMotor.class, "claw_left");
        clawRight = hardwareMap.get(DcMotor.class, "claw_right");
        clawRight.setDirection(DcMotor.Direction.REVERSE);
        clawLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
//        armLift.setTargetPosition(-300);
//        armLift.setPower(0.3);
//        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double fastMode = gamepad1.left_stick_button ? 0.85 : 0.60;
        if (fastMode > 0.60) {
            armLift.setTargetPosition(-300);
            armLift.setPower(0.6);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        int armEncoder = armLift.getCurrentPosition();

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1, 1) ;
        rightPower   = Range.clip(drive - turn, -1, 1) ;

        // Send calculated power to wheels
        rearLeftDrive.setPower(leftPower * fastMode);
        rearRightDrive.setPower(rightPower * fastMode);
        frontLeftDrive.setPower(leftPower * fastMode);
        frontRightDrive.setPower(rightPower * fastMode);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("arm pos", armLift.getCurrentPosition());

        // claw controls
        if (joystick_duck()) { // both bumpers = duck mode
            clawLeft.setPower(gamepad1.left_trigger);
            clawRight.setPower(-gamepad1.right_trigger);
        } else if (gamepad1.right_trigger > 0.0){
            // negative is in
            clawLeft.setPower(gamepad1.right_trigger);
            clawRight.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.0) { // left is in
            clawLeft.setPower(-gamepad1.left_trigger);
            clawRight.setPower(-gamepad1.left_trigger);
        } else {
            clawLeft.setPower(0);
            clawRight.setPower(0);
        }

//        telemetry.addData("armLimit", armLimit.getState());
//        if (!armLimit.getState()) {
//            armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }

        // SHARED SHIPPING HUB TIPPED - 20 pt!!!!!

        // switch (getGamepadButtons(gamepad1)
        // level 1 = -230
        // level 2 = -480
        // level 3 = -780

        // set heights 
        if (joystick_arm_ground()){
            telemetry.addData("button", "a"); // TODO change these to say ground/low/mid etc instead of button names
            armTargetRaw = -30;
            armState = STATE_HOLD;
        }
        if (joystick_arm_low()){
            telemetry.addData("button", "b");
            armTargetRaw = -300;
            armState = STATE_HOLD;
        }
        if (joystick_arm_mid()){
            telemetry.addData("button", "y");
            armTargetRaw = -600;
            armState = STATE_HOLD;
        }
        if (joystick_arm_high()){
            telemetry.addData("button", "x");
            armTargetRaw = -915;
            armState = STATE_HOLD;
        }


        // arm state machine
        if (armState == STATE_HOLD){
            // if duck mode buttons are pressed, then init duck mode
            if (joystick_duck()){
                armState = STATE_DUCK_INIT;
            }
        }

        if (armState == STATE_DUCK_INIT){
            armTargetRaw = -680;
            armState = STATE_DUCK_HOLD;
        }

        if (armState == STATE_DUCK_HOLD){
            // if duck mode buttons are released, then return to hold state. might need some debouncing..
            if (!joystick_duck()){
                armState = STATE_HOLD;
            }
        }

        // nudge
        if (joystick_nudge_down()){
            armTargetRaw -= 30;
        }
        if (joystic_nudge_up()){
            armTargetRaw += 30;
        }

        // trapezoidal profile
        armTargetSmooth = trap.smooth(armTargetRaw, (double) dt.time());
        dt.reset();

        // now command the motor!
        armLift.setTargetPosition(armTargetSmooth);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    // connect purpose to joystick buttons here
    private boolean joystick_arm_ground(){
        return gamepad1.a;
    }
    private boolean joystick_arm_low(){
        return gamepad1.b;
    }
    private boolean joystick_arm_mid(){
        return gamepad1.x;
    }
    private boolean joystick_arm_high(){
        return gamepad1.y;
    }
    private boolean joysick_nudge_up(){
        return gamepad1.dpad_up;
    }
    private boolean joystick_nudge_down(){
        return gamepad1.dpad_down;
    }

    private boolean joystick_duck(){
        return (gamepad1.right_trigger > 0.00 && gamepad1.left_trigger > 0.00);
    }

}
