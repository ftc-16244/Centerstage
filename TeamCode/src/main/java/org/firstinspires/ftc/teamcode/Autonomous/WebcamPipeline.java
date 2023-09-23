package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class WebcamPipeline extends OpenCvPipeline {

    //Outputs
    private Mat resizeImageOutput = new Mat();
    private Mat rgbThresholdOutput = new Mat();
    private Mat cvErodeOutput = new Mat();
    private Mat cvDilateOutput = new Mat();
    private Mat maskOutput = new Mat();
    private MatOfKeyPoint findBlobsOutput = new MatOfKeyPoint();

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    @Override
    public Mat processFrame(Mat source0) {
        // Step Resize_Image0:
        Mat resizeImageInput = source0;
        double resizeImageWidth = 640.0;
        double resizeImageHeight = 480.0;
        int resizeImageInterpolation = Imgproc.INTER_AREA;
        resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

        // Step RGB_Threshold0:
        Mat rgbThresholdInput = resizeImageOutput;
        double[] rgbThresholdRed = {91.72661870503607, 255.0};
        double[] rgbThresholdGreen = {0.0, 74.41126279863478};
        double[] rgbThresholdBlue = {0.0, 74.41126279863482};
        rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, rgbThresholdOutput);

        // Step CV_erode0:
        Mat cvErodeSrc = rgbThresholdOutput;
        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(-1, -1);
        double cvErodeIterations = 6.0;
        int cvErodeBordertype = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

        // Step CV_dilate0:
        Mat cvDilateSrc = cvErodeOutput;
        Mat cvDilateKernel = new Mat();
        Point cvDilateAnchor = new Point(-1, -1);
        double cvDilateIterations = 30.0;
        int cvDilateBordertype = Core.BORDER_CONSTANT;
        Scalar cvDilateBordervalue = new Scalar(-1);
        cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

        // Step Mask0:
        Mat maskInput = resizeImageOutput;
        Mat maskMask = cvDilateOutput;
        mask(maskInput, maskMask, maskOutput);

        // Step Find_Blobs0:
        Mat findBlobsInput = maskOutput;
        double findBlobsMinArea = 500.0;
        double[] findBlobsCircularity = {0.5593525179856115, 1.0};
        boolean findBlobsDarkBlobs = false;
        return findBlobs(findBlobsInput);
    }

    public Mat resizeImageOutput() {
        return resizeImageOutput;
    }

    public Mat rgbThresholdOutput() {
        return rgbThresholdOutput;
    }

    public Mat cvErodeOutput() {
        return cvErodeOutput;
    }

    public Mat cvDilateOutput() {
        return cvDilateOutput;
    }

    public Mat maskOutput() {
        return maskOutput;
    }

    public MatOfKeyPoint findBlobsOutput() {
        return findBlobsOutput;
    }

    private void resizeImage(Mat input, double width, double height, int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue, Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
        Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
                new Scalar(red[1], green[1], blue[1]), out);
    }

    private void cvErode(Mat src, Mat kernel, Point anchor, double iterations, int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1, -1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int) iterations, borderType, borderValue);
    }

    private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations, int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1, -1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.dilate(src, dst, kernel, anchor, (int) iterations, borderType, borderValue);
    }

    private void mask(Mat input, Mat mask, Mat output) {
        mask.convertTo(mask, CvType.CV_8UC1);
        Core.bitwise_xor(output, output, output);
        input.copyTo(output, mask);
    }

    private MatOfPoint findBlobs(Mat input) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Core.extractChannel(input, input, 2);
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return findLargestContour(contours);
    }
    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        return largestContour;
    }
}

