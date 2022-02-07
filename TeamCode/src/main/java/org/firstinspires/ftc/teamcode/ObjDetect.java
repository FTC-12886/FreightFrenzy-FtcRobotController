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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

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

public class ObjDetect {
    public static double RIGHT_BOUNDARY = 0.67;
    public static double LEFT_BOUNDARY = 0.33;
    public static double WIDTH_IGNORE = 1;
    public static double HEIGHT_IGNORE = 1;

    private final TFObjectDetector tfod;
    private ProcessingParameters processingParameters;

    public ObjDetect(TFObjectDetector tfod) {
        this.tfod = tfod;
        tfod.activate();
    }

    public ObjDetect(TFObjectDetector tfod, ProcessingParameters processingParameters) {
        this(tfod);
        this.processingParameters = processingParameters;
    }

    public List<RecognitionPercent> getRecognitions() {
        List<Recognition> recognitions = tfod.getRecognitions();
        if (recognitions == null) {
            return null;
        }
        List<RecognitionPercent> percentRecognitions = new ArrayList<>();
        for (Recognition recognition : recognitions) {
            percentRecognitions.add(new RecognitionPercent(recognition));
        }

        return percentRecognitions;
    }

    public List<RecognitionPercent> getApprovedRecognitions() {
        List<RecognitionPercent> recognitions = getRecognitions();
        if (recognitions == null) {
            return null;
        }
        List<RecognitionPercent> approvedRecognitions = new ArrayList<>();
        for (RecognitionPercent recognition : recognitions) {
            if (recognition.getWidth() < processingParameters.widthIgnore && recognition.getHeight() < processingParameters.heightIgnore) {
                approvedRecognitions.add(recognition);
            }
        }

        return approvedRecognitions;
    }

    public RecognitionPercent getAverageApprovedRecognitions() {
        List<RecognitionPercent> recognitions = getApprovedRecognitions();
        if (recognitions == null) {
            return null;
        }
        int i = 0;
        float left = 0, right = 0, top = 0, bottom = 0;
        float confidence = 0;
        double angle = 0;

        for (RecognitionPercent recognition : recognitions) {
            left += recognition.getLeft();
            right += recognition.getRight();
            top += recognition.getTop();
            bottom += recognition.getBottom();
            confidence += recognition.getConfidence();
            angle += recognition.estimateAngleToObject(AngleUnit.RADIANS);
            i++;
        }
        left /= i;
        right /= i;
        top /= i;
        bottom /= i;
        confidence /= i;
        angle /= i;
        return new RecognitionPercent(new RecognitionStore(left, right, top, bottom, confidence, 1, 1, angle, AngleUnit.RADIANS));
    }

    public int getPosition() {
        RecognitionPercent recognition = getAverageApprovedRecognitions();
        if (recognition == null)
            return 0;
        if (recognition.getX() < processingParameters.leftBoundary)
            return 1;
        else if (recognition.getX() < processingParameters.rightBoundary)
            return 2;
        else if (recognition.getX() > processingParameters.rightBoundary)
            return 3;
        else
            return -1;
    }

    public static class ProcessingParameters {
        public double rightBoundary = RIGHT_BOUNDARY;
        public double leftBoundary = LEFT_BOUNDARY;
        public double widthIgnore = WIDTH_IGNORE;
        public double heightIgnore = HEIGHT_IGNORE;
    }

    public static class RecognitionPercent implements org.firstinspires.ftc.robotcore.external.tfod.Recognition {
        private final Recognition recognition;
        @Override
        public String getLabel() {
            return recognition.getLabel();
        }

        @Override
        public float getConfidence() {
            return recognition.getConfidence();
        }

        @Override
        public float getLeft() {
            return recognition.getLeft() / recognition.getImageWidth();
        }

        @Override
        public float getRight() {
            return recognition.getRight() / recognition.getImageWidth();
        }

        @Override
        public float getTop() {
            return recognition.getTop() / recognition.getImageHeight();
        }

        @Override
        public float getBottom() {
            return recognition.getBottom() / recognition.getImageHeight();
        }

        @Override
        public float getWidth() {
            return Math.abs(getRight() - getLeft());
        }

        @Override
        public float getHeight() {
            return Math.abs(getBottom() - getTop());
        }

        public float getX() {
            return (getLeft() + getRight()) / 2;
        }

        public float getY() {
            return (getTop() + getBottom()) / 2;
        }

        public Recognition getOriginalRecognition() {
            return recognition;
        }

        @Override
        public int getImageWidth() {
            return 1;
        }

        @Override
        public int getImageHeight() {
            return 1;
        }

        @Override
        public double estimateAngleToObject(AngleUnit angleUnit) {
            return recognition.estimateAngleToObject(angleUnit);
        }

        public RecognitionPercent(Recognition recognition) {
            this.recognition = recognition;
        }

    }

    public static class RecognitionStore implements Recognition{
        private final float left;
        private final float right;
        private final float top;
        private final float bottom;
        private final float confidence;
        private final int imageWidth;
        private final int imageHeight;
        private final double angleInRadians;

        @Override
        public String getLabel() {
            return null;
        }

        @Override
        public float getConfidence() {
            return confidence;
        }

        @Override
        public float getLeft() {
            return left;
        }

        @Override
        public float getRight() {
            return right;
        }

        @Override
        public float getTop() {
            return top;
        }

        @Override
        public float getBottom() {
            return bottom;
        }

        @Override
        public float getWidth() {
            return Math.abs(getRight() - getLeft());
        }

        @Override
        public float getHeight() {
            return Math.abs(getBottom() - getTop());
        }

        @Override
        public int getImageWidth() {
            return imageWidth;
        }

        @Override
        public int getImageHeight() {
            return imageHeight;
        }

        @Override
        public double estimateAngleToObject(AngleUnit angleUnit) {
            return angleUnit.fromRadians(angleInRadians);
        }

        public RecognitionStore(float left, float right, float top, float bottom, float confidence, int imageWidth, int imageHeight, double angle, AngleUnit angleUnit) {
            this.left = left;
            this.right = right;
            this.top = top;
            this.bottom = bottom;
            this.confidence = confidence;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.angleInRadians = angleUnit.toRadians(angle);
        }
    }

}
