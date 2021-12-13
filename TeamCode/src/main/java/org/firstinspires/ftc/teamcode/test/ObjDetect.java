/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Config
@Autonomous
public class ObjDetect extends LinearOpMode {
    public static double RIGHT_BOUNDARY = 0.67;
    public static double LEFT_BOUNDARY = 0.33;
    public static double WIDTH_IGNORE = 0.25;
    public static double HEIGHT_IGNORE = 0.5;
    public static double SIZE_X = 50;
    public static double SIZE_Y = 50;
    public static boolean SHOW_IGNORE = false;
    private static final String[] COLORS = new String[]{"blue", "green", "yellow", "purple", "teal"};
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
    private static final String TFOD_MODEL_FILE = AppUtil.FIRST_FOLDER + "/tflitemodels/TeamElement.tflite";
    private static final String[] LABELS = {
            "Team"
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

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 4.0 / 3.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            tfod.shutdown();
            vuforia.close();
            return;
        }

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                TelemetryPacket graph = new TelemetryPacket();
                addLines(graph.fieldOverlay(), LEFT_BOUNDARY, RIGHT_BOUNDARY, 1, "black");
                addRectangle(graph.fieldOverlay(), 0, 0, 1, 1, 1, "black"); // drawing area
                if (SHOW_IGNORE) {
                    addRectangle(graph.fieldOverlay(), 0, 0, WIDTH_IGNORE, HEIGHT_IGNORE, 1, "red"); // ignore area
                    addCircle(graph.fieldOverlay(), WIDTH_IGNORE / 2, HEIGHT_IGNORE / 2, 1, "red");
                }
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        double sumTop = 0;
                        double sumLeft = 0;
                        double sumBottom = 0;
                        double sumRight = 0;
                        double sumX = 0;
                        double sumY = 0;
                        int numNonIgnore = 0;
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            boolean ignore = (recognition.getRight() - recognition.getLeft()) / recognition.getImageWidth() > WIDTH_IGNORE ||
                                    (recognition.getBottom() - recognition.getTop()) / recognition.getImageHeight() > HEIGHT_IGNORE;
                            numNonIgnore += ignore ? 0 : 1;


                            double top = recognition.getTop() / recognition.getImageHeight();
                            sumTop += !ignore ? top : 0;
                            double left = recognition.getLeft() / recognition.getImageWidth();
                            sumLeft += !ignore ? left : 0;
                            double bottom = recognition.getBottom() / recognition.getImageHeight();
                            sumBottom += !ignore ? bottom : 0;
                            double right = recognition.getRight() / recognition.getImageWidth();
                            sumRight += !ignore ? right : 0;

                            double x = (left + right) / 2;
                            sumX += !ignore ? x : 0;
                            double y = (top + bottom) / 2;
                            sumY += !ignore ? y : 0;

                            String color = ignore ? "red" : COLORS[i];
                            if (!ignore || SHOW_IGNORE) {
                                addCircle(graph.fieldOverlay(), x, y, 1, color);
                                addRectangle(graph.fieldOverlay(), left, top, right, bottom, 1, color);
                            }
                            telemetry.addData(String.format(new Locale("en", "US"),
                                    "label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format(new Locale("en", "US"),
                                            "  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format(new Locale("en", "US"),
                                            "  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());


                            telemetry.addData(String.format(new Locale("en", "US"),
                                    "  x,y (%d)", i), "%.03f , %.03f",
                                    x, y);

                            if (x < LEFT_BOUNDARY) {
                                telemetry.addData("   side","left");
                            } else if (x < RIGHT_BOUNDARY) {
                                telemetry.addData("   side", "middle");
                            } else {
                                telemetry.addData("   side", "right");
                            }
                            telemetry.addData("   ignore", ignore);
                            i++;
                        }
                        if (numNonIgnore != 0) {
                            addCircle(graph.fieldOverlay(), sumX / numNonIgnore, sumY / numNonIgnore, 1, "black");
                            addRectangle(graph.fieldOverlay(), sumLeft / numNonIgnore, sumTop / numNonIgnore, sumRight / numNonIgnore, sumBottom / numNonIgnore, 1, "black");
                        }
                        telemetry.addData("sumX", sumX);
                        telemetry.addData("sumY", sumY);
                        telemetry.addData("numNonIgnore", numNonIgnore);
                        FtcDashboard.getInstance().sendTelemetryPacket(graph);
                    }
                }
                telemetry.update();
            }
        }
        tfod.shutdown();
        vuforia.close();
    }



    private void addLines(Canvas canvas, double leftBoundary, double rightBoundary, int width, String color) {
        canvas.setStrokeWidth(width);
        canvas.setStroke(color);
        canvas.strokeLine(SIZE_Y, -convertX(leftBoundary), -SIZE_Y, -convertX(leftBoundary));
        canvas.strokeLine(SIZE_Y, -convertX(rightBoundary), -SIZE_Y, -convertX(rightBoundary));
    }

    private void addCircle(Canvas canvas, double x, double y, double radius, String color) {
        canvas.setFill(color);
        x = convertX(x);
        y = convertY(y);
        canvas.fillCircle(y, -x, radius);
    }

    private void addRectangle(Canvas canvas, double x1, double y1, double x2, double y2, int width, String color) {
        canvas.setStrokeWidth(width);
        canvas.setStroke(color);
        x1 = convertX(x1);
        y1 = convertY(y1);
        x2 = convertX(x2);
        y2 = convertY(y2);
        double x3 = x2;
        double y3 = y1;
        double x4 = x1;
        double y4 = y2;
        canvas.strokePolygon(new double[]{y1, y3, y2, y4, y1}, new double[]{-x1, -x3, -x2, -x4, -x1});
    }

    private double convertX(double x) {
        return (x - SIZE_X) + 2 * SIZE_X * x;
    }

    private double convertY(double y) {
        return (y + SIZE_Y) - 2 * SIZE_Y * y;
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
        //parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        // MUST CREATE WITH R.id.tfodMonitorViewId if camera stream is to work
        int tfodMonitorViewId = R.id.tfodMonitorViewId;
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        FtcDashboard.getInstance().startCameraStream(tfod, 0);
    }
}
