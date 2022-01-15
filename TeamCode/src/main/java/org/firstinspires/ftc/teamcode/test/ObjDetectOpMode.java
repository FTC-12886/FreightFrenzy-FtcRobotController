package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RunShellCommand;
import com.qualcomm.robotcore.util.WebHandlerManager;
import com.qualcomm.robotcore.util.WebServer;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebInfo;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.WebSocketManager;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="Obj Detect Test", group="ML")
@Disabled
public class ObjDetectOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String tfodModelFile = AppUtil.FIRST_FOLDER + "/tflitemodels/TeamElement.tflite";
        String[] labels = {
                "Team"
        };

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "ASg2QBr/////AAABmU3Gzjd/akXrk1NzMQrLNgN6wxZIJ3H7AHf8eU6cL+4hcspa6m1glKBeuuSXaELDtK5J81Ewk7+bYxWFk66Y8qupXK8Hqo81er+2T7R7gfZ5O+dCnJpBmU394oA0PrT2L1qAn3ArLA9bkjNM7xauWiff4YtcuSyDBbBGcMJz1BUDMSJ5az94/XlX+d3ATUBiR3T82RSPXZfv6dn+TvIDr1DqLNwgQnzgTPWZwgITgvAAscBjxETX4CgzThrOShqVkKxAtWOyj+uuU53UIhNHsVMEsJuafMqg+Mhkp6c/+VP6LoFPDJwGwdMxrFByCf2GAKkxmWFTQzreHtwVsN2u5O8wXOGlL5WCi3L1R5Iw7MaW";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = R.id.tfodMonitorViewId;
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(tfodModelFile, labels);

        ObjDetect.ProcessingParameters processingParameters = new ObjDetect.ProcessingParameters();
        ObjDetect objDetect = new ObjDetect(tfod, processingParameters);

        if (isStopRequested()) {
            tfod.shutdown();
            vuforia.close();
        }

        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        double integral = 0;
        int i = 0;
        while (opModeIsActive()) {
            int position = objDetect.getPosition();
            if (position > 0) {
                integral += position;
                i++;
            }

            telemetry.addData("position", position);
            telemetry.addData("average", integral/i);
            telemetry.addData("time", timer.seconds());

            if (timer.seconds() >= 5) {
                timer.reset();
                integral = 0;
                i = 0;
            }
            telemetry.update();
        }

        tfod.deactivate();
        tfod.shutdown();
        vuforia.close();
    }
}
