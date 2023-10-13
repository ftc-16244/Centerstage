package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.CENTER_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.LEFT_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.CENTER_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.LEFT_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.RIGHT_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.RIGHT_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.RED_LOW_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.RED_HIGH_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.BLUE_LOW_HSV;
import static org.firstinspires.ftc.teamcode.Pipelines.ConstantsForPositions.BLUE_HIGH_HSV;
// Credit to WolfCorpFTC team # 12525 for the original file.
// 16244 modified for webcam and for the Centerstage team element

public class WebcamPipeline extends OpenCvPipeline {
    Prop location;
    Telemetry telemetry;
    StartPosition startPosition;
    Scalar lowHSV;
    Scalar highHSV;
    Mat mat = new Mat(); // Mat is a matrix

    static double PERCENT_COLOR_THRESHOLD = 0.15;

    public WebcamPipeline(Telemetry t, StartPosition position) {
        telemetry = t;
        startPosition = position;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Rect NONCENTER_ROI;
        Rect CENTER_ROI;
        Prop undetectableLocation;
        Prop detectableNoncenter;

        if (startPosition == StartPosition.RED_AUD) {
            NONCENTER_ROI = LEFT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
            undetectableLocation = Prop.LEFT;
            detectableNoncenter = Prop.RIGHT;
            lowHSV = RED_LOW_HSV;
            highHSV = RED_HIGH_HSV;
        }
        else if (startPosition == StartPosition.BLUE_AUD) {
            NONCENTER_ROI = RIGHT_ROI_BLUE;
            CENTER_ROI = CENTER_ROI_BLUE;
            undetectableLocation = Prop.RIGHT;
            detectableNoncenter = Prop.LEFT;
            lowHSV = BLUE_LOW_HSV;
            highHSV = BLUE_HIGH_HSV;
        }
        else if (startPosition == StartPosition.RED_STAGE) {
            NONCENTER_ROI = RIGHT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
            undetectableLocation = Prop.RIGHT;
            detectableNoncenter = Prop.LEFT;
            lowHSV = RED_LOW_HSV;
            highHSV = RED_HIGH_HSV;
        }
        else if (startPosition == StartPosition.BLUE_STAGE) {
            NONCENTER_ROI = LEFT_ROI_BLUE;
            CENTER_ROI = CENTER_ROI_BLUE;
            undetectableLocation = Prop.LEFT;
            detectableNoncenter = Prop.RIGHT;
            lowHSV = BLUE_LOW_HSV;
            highHSV = BLUE_HIGH_HSV;
        }
        else {
            throw new IllegalArgumentException("Invalid start position passed to pipeline!");
        }

        // takes the values that are between lowHSV and highHSV only
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat noncenter = mat.submat(NONCENTER_ROI); //sub matrices of mat
        Mat right = mat.submat(CENTER_ROI);

        // if a pixel is deemed to be between the low and high HSV range OpenCV makes it white
        // white is given a value of 255. This way the new image is just grayscale where 0 is black
        // and 255 is white. Below all elements of the sub matrix are added up and divided by the area and then by 255.
        // This essentially calculates the ratio of identified pixels to those not identified. The
        //higher the value the more detection.

        double noncenterValue = Core.sumElems(noncenter).val[0] / NONCENTER_ROI.area() / 255;
        double centerValue = Core.sumElems(right).val[0] / CENTER_ROI.area() / 255;

        noncenter.release(); // frees up memory
        right.release();

        telemetry.addData(detectableNoncenter + " percentage", Math.round(noncenterValue * 100) + "%");
        telemetry.addData("CENTER percentage", Math.round(centerValue * 100) + "%");

        boolean inNoncenterPosition = noncenterValue > PERCENT_COLOR_THRESHOLD; // sets a limit to compare to so small objects don't accidentally trigger
        boolean inCenterPosition = centerValue > PERCENT_COLOR_THRESHOLD;
        location = undetectableLocation;
        if(inNoncenterPosition) location = detectableNoncenter;
        else if(inCenterPosition) location = Prop.CENTER;
        telemetry.addData("Detected position: ", String.valueOf(getPropLocation()));
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNone = new Scalar(255, 0, 0); // color scheme for targeting boxes drawn on the display
        Scalar colorTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, NONCENTER_ROI, location == detectableNoncenter? colorTSE:colorNone); // the target boxes surround the ROI's
        Imgproc.rectangle(mat, CENTER_ROI, location == Prop.CENTER? colorTSE:colorNone);

        return mat;
    }
    public Prop getPropLocation() {
        return location;
    }
}