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


public class WebcamPipelineNew extends OpenCvPipeline{
    Telemetry telemetry;
    Mat mat = new Mat(); // Mat is a matrix


    static double PERCENT_COLOR_THRESHOLD = 0.2;

    // telemetry is part of a LinearOpmde. Since this pipeline does not extend a LinearOpmode
    // telemetry has to be added in the constructor for SkystonePipeline to be able to use it.

    // Constructor
    public WebcamPipelineNew(Telemetry t) {
        telemetry = t;

    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // converting RGB to HSV color scheme
        // Normal HSV has HUE values from 0 to 360. Open CV uses 0 to 180,
        // Normal HSV has Saturation and Value from 0 to 100. Open CV scales that 100 from 0 to 255
        // HUE's for reference
        // Red 0
        // Orange   25 deg or 12.5 in Open VC
        // Yellow   60 or 30 in Open CV
        // Green    120 or 60 in open CV
        // Cyan     180 or 90 in Open CV
        // Blue     240 or 120 in Open CV
        // Purple   280 or 140 in Open CV
        // Pink     300 or 150 in open CV
        // Red      360 or 180 in openCV

        Rect LEFT_ROI = null; // the selected ROI based on alliance and start position
        Rect RIGHT_ROI = null;

        // ROI's for the Blue Carousel where we line up in front of the left and center bar codes
        Rect LEFT_ROI_A = new Rect(
                new Point(75, 100),
                new Point(135, 140));
        Rect RIGHT_ROI_A = new Rect(
                new Point(230, 100),
                new Point(290, 140));


        // ROI's for the Red Carousel where we  up in front of the center and right bar codes
        Rect LEFT_ROI_B = new Rect(
                new Point(0, 115),
                new Point(60, 155));
        Rect RIGHT_ROI_B = new Rect(
                new Point(180, 115),
                new Point(240, 155));


        // Blue carousel and red warehouse have the robot in front of the left and center barcode.
        // Red carousel and blue warehouse have the robot in front of the center and right barcode
        // Due to the asymmetry of camera placement, different ROI's are needed for the two generic
        // starting positions.

        // Use a webcam and the GRIP software to tune these values off the robot. It will save a lot of time.
        // GRIP is a free download.
        // For Ol "Party Hat" TSE that is mre Teal Green
        //Scalar lowHSV = new Scalar(69, 59,  57); // for cyan smaller H allows more green
        //Scalar highHSV = new Scalar(104, 300, 179); // for cyan larger H allows more blue. If too much blue is allowed the blue floor tape is detected.

        // For new multitude of TSE's that are basically John Deere Green
        Scalar lowHSV = new Scalar(42, 117,  0);
        Scalar highHSV = new Scalar(100, 255, 255);

        // takes the values that are between lowHSV and highHSV only
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI); //sub matrices of mat
        Mat right = mat.submat(RIGHT_ROI);

        // if a pixel is deemed to be between the low and high HSV range OpenCV makes it white
        // white is given a value of 255. This way the new image is just grayscale where 0 is black
        // and 255 is white. Below all elements of the sub matrix are added up and divided by the area and then by 255.
        // This essentially calculates the ratio of identified pixels to those not identified. The
        //higher the value the more detection.

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release(); // frees up memory
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean TSERobotLeft = leftValue > PERCENT_COLOR_THRESHOLD; // sets a limit to compare to so small objects don't accidentally trigger
        boolean TSERobotRight = rightValue > PERCENT_COLOR_THRESHOLD;
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNone = new Scalar(255, 0, 0); // color scheme for targeting boxes drawn on the display
        Scalar colorTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, barcode == Barcode.LEFT? colorTSE:colorNone); // the target boxes surround the ROI's
        Imgproc.rectangle(mat, RIGHT_ROI, barcode == Barcode.RIGHT? colorTSE:colorNone);

        return mat;
    }

}