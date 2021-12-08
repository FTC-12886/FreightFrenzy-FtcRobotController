
package org.firstinspires.ftc.teamcode.skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Hub Test")
@Disabled
public class AutoSketch extends OpMode {

    private DcMotor rearRight = null;
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;

    private DcMotor frontLauncher = null;
    private DcMotor rearLauncher = null;
    private DcMotor intakeMotor = null;

    private DistanceSensor leftDist = null;
    private DistanceSensor rightDist = null;

    private BNO055IMU imu;

    private double power = 0;
    private Orientation angles;

    @Override
    public void init() {
        rearRight  = hardwareMap.get(DcMotor.class, "RearRight");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft" );
        rearLeft = hardwareMap.get(DcMotor.class, "RearLeft");

        frontLauncher = hardwareMap.get(DcMotor.class, "FrontLauncher");
        rearLauncher = hardwareMap.get(DcMotor.class, "RearLauncher");
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        rearRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
        rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");

        gyroInit();
    }


    @Override
    public void init_loop() {
        gyroLoop();
        telemetry.addData("MOTORS", "READY");
        telemetry.addData("LEFTDIST", "READY");
        telemetry.addData("RIGHTDIST", "READY");
    }

    @Override
    public void loop() {
        gyroLoop();
        stateTest(-0.5);
        rampTest(-1);
        telemetry.addData("LEFTDIST", leftDist.getDistance(DistanceUnit.CM));
        telemetry.addData("RIGHTDIST", rightDist.getDistance(DistanceUnit.CM));
        telemetry.addData("MOTOR POWER", power);
    }

    @Override
    public void stop() {

    }

    private void rampTest(double power) {
        // frontLauncher.setPower(power);
        // rearLauncher.setPower(power);
        intakeMotor.setPower(0.7 * power);
    }

    private void stateTest(double direction) {
        double distance = leftDist.getDistance(DistanceUnit.CM);
        if (distance > 60) {
            power = 0.75;
        }
        else if (60 > distance && distance > 40) {
            power = 0.55;
        }
        else if (40 > distance && distance > 20) {
            power = 0.35;
        }
        else if (20 > distance) {
            power = 0.15;
        }
        power = power * direction;
        rearRight.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(power);
        rearLeft.setPower(power);
    }

    private void gyroInit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }

    private void gyroLoop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // telemetry.addData("ANGLE", angles);
        telemetry.addData("HEADING", angles.firstAngle);
    }

}