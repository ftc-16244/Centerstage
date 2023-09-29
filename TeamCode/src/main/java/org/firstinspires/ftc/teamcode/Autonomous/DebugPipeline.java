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
    Barcode barcode = Barcode.RIGHT;
    Telemetry telemetry;
    Mat mat = new Mat(); // Mat is a matrix


    static double PERCENT_COLOR_THRESHOLD = 0.2;

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

        left.release(); // frees up memory
        right.release();

        telemetry.update();

        Scalar colorNone = new Scalar(255, 0, 0); // color scheme for targeting boxes drawn on the display
        Scalar colorTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, barcode == Barcode.LEFT? colorTSE:colorNone); // the target boxes surround the ROI's
        Imgproc.rectangle(mat, RIGHT_ROI, barcode == Barcode.RIGHT? colorTSE:colorNone);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

        return mat;
    }

}