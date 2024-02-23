package org.firstinspires.ftc.teamcode.Pipelines;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Pipelines.Constants.BLUE_HIGH_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.BLUE_LOW_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.CENTER_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.CENTER_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.LEFT_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.LEFT_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RED_HIGH_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RED_LOW_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RIGHT_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RIGHT_ROI_RED;

// Credit to WolfCorpFTC team # 12525 for the original file.
// 16244 modified for webcam and for the Centerstage team element
// Also heavily modified for telemetry, multiple ROIs, and many other things

public class Pipeline extends OpenCvPipeline {
    boolean telemetryEnabled = true;
    Prop location;
    Telemetry telemetry;
    StartPosition startPosition;
    Scalar lowHSV;
    Scalar highHSV;
    Mat mat = new Mat(); // Mat is a matrix
    Mat output = new Mat();
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pipelineReady = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
    private static final int PERCENT_COLOR_THRESHOLD = 10;
    public Pipeline(Telemetry t, StartPosition position, RevBlinkinLedDriver blinkin) {
        telemetry = t;
        startPosition = position;
        blinkinLedDriver = blinkin;
    }
    @Override
    public Mat processFrame(Mat input) { // DON'T under any circumstances do anything that has the smallest possibility of changing input
        // for example, doing output = input; then doing Imgproc.cvtColor(output, output, Imgproc.COLOR_BGRA2BGR); breaks it
        System.out.println("Process frame called");
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, output, Imgproc.COLOR_BGRA2BGR);
        Rect LEFT_ROI;
        Rect CENTER_ROI;
        Rect RIGHT_ROI;

        if (startPosition == StartPosition.RED_AUD) {
            LEFT_ROI = LEFT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
            RIGHT_ROI = RIGHT_ROI_RED;
            lowHSV = RED_LOW_HSV;
            highHSV = RED_HIGH_HSV;
        }
        else if (startPosition == StartPosition.BLUE_AUD) {
            LEFT_ROI = LEFT_ROI_BLUE;
            CENTER_ROI = CENTER_ROI_BLUE;
            RIGHT_ROI = RIGHT_ROI_BLUE;
            lowHSV = BLUE_LOW_HSV;
            highHSV = BLUE_HIGH_HSV;
        }
        else if (startPosition == StartPosition.RED_STAGE) {
            LEFT_ROI = LEFT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
            RIGHT_ROI = RIGHT_ROI_RED;
            lowHSV = RED_LOW_HSV;
            highHSV = RED_HIGH_HSV;
        }
        else if (startPosition == StartPosition.BLUE_STAGE) {
            LEFT_ROI = LEFT_ROI_BLUE;
            CENTER_ROI = CENTER_ROI_BLUE;
            RIGHT_ROI = RIGHT_ROI_BLUE;
            lowHSV = BLUE_LOW_HSV;
            highHSV = BLUE_HIGH_HSV;
        }
        else {
            throw new IllegalArgumentException("Invalid start position passed to pipeline!");
        }

        // takes the values that are between lowHSV and highHSV only
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI); //sub matrices of mat
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        // if a pixel is deemed to be between the low and high HSV range OpenCV makes it white
        // white is given a value of 255. This way the new image is just grayscale where 0 is black
        // and 255 is white. Below all elements of the sub matrix are added up and divided by the area and then by 255.
        // This essentially calculates the ratio of identified pixels to those not identified. The
        //higher the value the more detection.

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 2.55;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 2.55;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 2.55;

        left.release();
        center.release();
        right.release();

        location = null;
        if (leftValue > PERCENT_COLOR_THRESHOLD) location = Prop.LEFT;
        else if (centerValue > PERCENT_COLOR_THRESHOLD) location = Prop.CENTER;
        else if (rightValue > PERCENT_COLOR_THRESHOLD) location = Prop.RIGHT;

        if (!(location == null)) telemetry.addData("Detected position: ", String.valueOf(getPropLocation()));
        else telemetry.addLine("No prop detected.");

        if (telemetryEnabled) {
            telemetry.addData("LEFT percentage",  Math.round(leftValue * 10) / 10 + "%");
            telemetry.addData("CENTER percentage",Math.round(centerValue * 10) / 10 + "%");
            telemetry.addData("RIGHT percentage", Math.round(rightValue * 10) / 10 + "%");
            telemetry.update();
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Core.bitwise_and(output, mat, output);

        Scalar colorNone = new Scalar(255, 0, 0); // color scheme for targeting boxes drawn on the display
        Scalar colorDetection = new Scalar(0, 255, 0);

        Imgproc.rectangle(output, LEFT_ROI, location == Prop.LEFT? colorDetection:colorNone); // the target boxes surround the ROI's
        Imgproc.rectangle(output, CENTER_ROI, location == Prop.CENTER? colorDetection:colorNone);
        Imgproc.rectangle(output, RIGHT_ROI, location == Prop.RIGHT? colorDetection:colorNone);

        blinkinLedDriver.setPattern(pipelineReady);
        System.out.println("Process frame done");

        return output;
    }
    public Prop getPropLocation() {
        return location;
    }
    public void toggleTelemetry() {
        telemetryEnabled = !telemetryEnabled;
    }
}