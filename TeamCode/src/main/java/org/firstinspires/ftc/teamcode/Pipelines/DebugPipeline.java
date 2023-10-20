package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Pipelines.Constants.CENTER_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.LEFT_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.CENTER_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.LEFT_ROI_BLUE;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RIGHT_ROI_RED;
import static org.firstinspires.ftc.teamcode.Pipelines.Constants.RIGHT_ROI_BLUE;
// Credit to WolfCorpFTC team # 12525 for the original file.
// 16244 modified for webcam and for the Centerstage team element

public class DebugPipeline extends OpenCvPipeline {
    StartPosition startPosition;
    Mat mat = new Mat(); // Mat is a matrix

    public DebugPipeline(StartPosition position) {
        startPosition = position;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Rect NONCENTER_ROI;
        Rect CENTER_ROI;

        if (startPosition == StartPosition.RED_AUD) {
            NONCENTER_ROI = LEFT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
        }
        else if (startPosition == StartPosition.BLUE_AUD) {
            NONCENTER_ROI = LEFT_ROI_BLUE;
            CENTER_ROI = CENTER_ROI_BLUE;
        }
        else if (startPosition == StartPosition.RED_STAGE) {
            NONCENTER_ROI = RIGHT_ROI_RED;
            CENTER_ROI = CENTER_ROI_RED;
        }
        else if (startPosition == StartPosition.BLUE_STAGE) {
            NONCENTER_ROI = RIGHT_ROI_BLUE;
            CENTER_ROI = CENTER_ROI_BLUE;
        }
        else {
            throw new IllegalArgumentException("Invalid start position passed to pipeline!");
        }

        Scalar colorNone = new Scalar(255, 0, 0); // color scheme for targeting boxes drawn on the display

        Imgproc.rectangle(mat, NONCENTER_ROI, colorNone); // the target boxes surround the ROI's
        Imgproc.rectangle(mat, CENTER_ROI, colorNone);

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

        return mat;
    }
}