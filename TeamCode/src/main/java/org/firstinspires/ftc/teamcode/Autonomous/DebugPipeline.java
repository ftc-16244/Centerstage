package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
// Credit to WolfCorpFTC team # 12525 for the original file.
// 16244 modified for webcam and for the Centerstage team element


public class DebugPipeline extends OpenCvPipeline {
    Prop location = Prop.RIGHT;
    Telemetry telemetry;
    Mat mat = new Mat(); // Mat is a matrix


    static double PERCENT_COLOR_THRESHOLD = 0.35;

    public DebugPipeline(Telemetry t) {
        telemetry = t;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Rect LEFT_ROI = new Rect(
                new Point(25, 70),
                new Point(100, 150));
        Rect RIGHT_ROI = new Rect(
                new Point(220, 50),
                new Point(285, 120));


        Mat left = mat.submat(LEFT_ROI); //sub matrices of mat
        Mat right = mat.submat(RIGHT_ROI);

        // if a pixel is deemed to be between the low and high HSV range OpenCV makes it white
        // white is given a value of 255. This way the new image is just grayscale where 0 is black
        // and 255 is white. Below all elements of the sub matrix are added up and divided by the area and then by 255.
        // This essentially calculates the ratio of identified pixels to those not identified. The
        //higher the value the more detection.

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release(); // frees up memory
        right.release();

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");

        boolean inLeftPosition = leftValue > PERCENT_COLOR_THRESHOLD; // sets a limit to compare to so small objects don't accidentally trigger
        boolean inCenterPosition = centerValue > PERCENT_COLOR_THRESHOLD;
        if(inLeftPosition) location = Prop.LEFT;
        else if(inCenterPosition) location = Prop.CENTER;
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNone = new Scalar(255, 0, 0); // color scheme for targeting boxes drawn on the display
        Scalar colorTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Prop.LEFT? colorTSE:colorNone); // the target boxes surround the ROI's
        Imgproc.rectangle(mat, RIGHT_ROI, location == Prop.CENTER? colorTSE:colorNone);

        return mat;
    }
    public Prop getPropLocation() {
        return location;
    }
}