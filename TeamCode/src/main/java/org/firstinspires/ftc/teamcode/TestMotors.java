package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="4WD Motor Test")

public class TestMotors extends OpMode {
    private DcMotor[] motorArray = new DcMotor[4];
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    int i = 0;
    private ElapsedTime elapsedTime = new ElapsedTime();
    @Override
    public void init() {
        motorArray[0] = hardwareMap.get(DcMotor.class, "rear_right_drive");
        motorArray[1] = hardwareMap.get(DcMotor.class, "rear_left_drive");
        motorArray[2] = hardwareMap.get(DcMotor.class, "front_right_drive");
        motorArray[3] = hardwareMap.get(DcMotor.class, "front_left_drive");

        telemetry.addData(">", "Press start to begin");
        telemetry.addData("rear right", "port 0");
        telemetry.addData("rear left", "port 1");
        telemetry.addData("front right", "port 2");
        telemetry.addData("front left", "port 3");
    }

    @Override
    public void loop() {
        DcMotor motor = motorArray[i%4];
        telemetry.addData("runtime", elapsedTime.milliseconds());
        telemetry.addData("motor", motor.getPortNumber());
        if ((i/4)%2 == 0) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        telemetry.addData("direction", motor.getDirection().toString());
        motor.setPower(1);
        if (elapsedTime.seconds() > 10) {
            i++;
            elapsedTime.reset();
        }


    }
}