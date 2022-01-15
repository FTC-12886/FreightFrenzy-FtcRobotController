package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.test.ObjDetect.SIZE_X;
import static org.firstinspires.ftc.teamcode.test.ObjDetect.SIZE_Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
@Disabled
public class TelemetryPacketTest extends LinearOpMode {
    public static double CIRCLE_X = 0;
    public static double CIRCLE_Y = 0;
    public static double RADIUS = 1;
    public static double LEFT_BOUNDARY = 0.33;
    public static double RIGHT_BOUNDARY = 0.67;
    public static double X1 = 0;
    public static double Y1 = 0;
    public static double X2 = 0.25;
    public static double Y2 = 0.25;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            addLines(packet.fieldOverlay(), LEFT_BOUNDARY, RIGHT_BOUNDARY, 1, "black");
            addCircle(packet.fieldOverlay(), CIRCLE_X, CIRCLE_Y, RADIUS, "red");
            addRectangle(packet.fieldOverlay(), X1, Y1, X2, Y2, 1, "blue");
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private void addLines(Canvas canvas, double leftBoundary, double rightBoundary, int width, String color) {
        canvas.setStrokeWidth(width);
        canvas.setStroke(color);
        canvas.strokeLine(SIZE_Y, -convertX(leftBoundary), -SIZE_Y,-convertX(leftBoundary));
        canvas.strokeLine(SIZE_Y, -convertX(rightBoundary), -SIZE_Y,-convertX(rightBoundary));
    }

    private void addCircle(Canvas canvas, double x, double y, double radius, String color) {
        canvas.setStroke(color);
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
        canvas.strokePolygon(new double[] {y1, y3, y2, y4, y1}, new double[] {-x1, -x3, -x2, -x4, -x1});
    }

    private double convertX(double x) {
        return (x-SIZE_X)+2*SIZE_X*x;
    }

    private double convertY(double y) {
        return (y+SIZE_Y)-2*SIZE_Y*y;
    }
    private void sendGraph(double x, double y, double radius) {
        TelemetryPacket graph = new TelemetryPacket();
        graph.fieldOverlay()
                .setFill("green")
                .fillCircle(y, -x, radius);
        dashboard.sendTelemetryPacket(graph);
    }


}
