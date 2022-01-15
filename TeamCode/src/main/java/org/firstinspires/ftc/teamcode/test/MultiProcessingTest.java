package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class MultiProcessingTest extends LinearOpMode {
    int started = 0;
    int init = 0;
    Runnable runnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                if (init >= 5000) {
                    init = 0;
                }
                telemetry.addData("Init", init++);
                telemetry.update();
            }
        }
    };
    @Override
    public void runOpMode() throws InterruptedException {
        Thread thread = new Thread(runnable);
        thread.start();

        waitForStart();
        while (opModeIsActive()) {
            if (started >= 5000) {
                started = 0;
            }
            telemetry.addData("Started", started++);
            telemetry.update();
        }

        thread.interrupt();
    }
}
