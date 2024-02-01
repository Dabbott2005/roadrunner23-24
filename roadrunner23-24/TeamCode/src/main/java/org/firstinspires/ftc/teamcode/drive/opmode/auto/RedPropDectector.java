package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedPropDectector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location{
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(426,720));
    static final Rect MID_ROI = new Rect(
            new Point(426,0),
            new Point(852,720));
    static final Rect RIGHT_ROI = new Rect(
            new Point(852,0),
            new Point(1280,720));
    static double PERCENT_COLOR_THRESHOLD = 0.2;
    public RedPropDectector(Telemetry t) {telemetry = t; }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 70, 50);
        Scalar highHSV = new Scalar(10, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MID_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MID_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        middle.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean propLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean propMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean propRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (propLeft) {
            location = Location.LEFT;
            telemetry.addData("Prop Location", "LEFT");

        }
        if (propMiddle) {
            location = Location.MIDDLE;
            telemetry.addData("Prop Location", "MIDDLE");

        }
        if (propRight) {
            location = Location.RIGHT;
            telemetry.addData("Prop Location", "RIGHT");


        } else {
            location = Location.RIGHT;
            telemetry.addData("Prop Location", "NOT DETECTED - RIGHT");

        }
        telemetry.update();

        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2BGR);

        Scalar redProp = new Scalar(255, 0,0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? redProp:redProp);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MIDDLE? redProp:redProp);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? redProp:redProp);

        return mat;
    }

    public Location getLocation() {
        return location;
    }

}
