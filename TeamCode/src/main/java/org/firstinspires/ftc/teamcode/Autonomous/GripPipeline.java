package org.firstinspires.ftc.teamcode.Autonomous;

class GripPipeline {

}
/*package org.firstinspires.ftc.teamcode.Autonomous;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import org.openftc.easyopencv.

public class GripPipeline {


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

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.

	public void process(Mat source0) {
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
		findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);

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



	private void resizeImage(Mat input, double width, double height,
		int interpolation, Mat output) {
		Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
	}


	private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
		Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
			new Scalar(red[1], green[1], blue[1]), out);
	}


	private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
		int borderType, Scalar borderValue, Mat dst) {
		if (kernel == null) {
			kernel = new Mat();
		}
		if (anchor == null) {
			anchor = new Point(-1,-1);
		}
		if (borderValue == null) {
			borderValue = new Scalar(-1);
		}
		Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
	int borderType, Scalar borderValue, Mat dst) {
		if (kernel == null) {
			kernel = new Mat();
		}
		if (anchor == null) {
			anchor = new Point(-1,-1);
		}
		if (borderValue == null){
			borderValue = new Scalar(-1);
		}
		Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}


	private void mask(Mat input, Mat mask, Mat output) {
		mask.convertTo(mask, CvType.CV_8UC1);
		Core.bitwise_xor(output, output, output);
		input.copyTo(output, mask);
	}

	private void findBlobs(Mat input, double minArea, double[] circularity,
		Boolean darkBlobs, MatOfKeyPoint blobList) {
		FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
		try {
			File tempFile = File.createTempFile("config", ".xml");

			StringBuilder config = new StringBuilder();

			config.append("<?xml version=\"1.0\"?>\n");
			config.append("<opencv_storage>\n");
			config.append("<thresholdStep>10.</thresholdStep>\n");
			config.append("<minThreshold>50.</minThreshold>\n");
			config.append("<maxThreshold>220.</maxThreshold>\n");
			config.append("<minRepeatability>2</minRepeatability>\n");
			config.append("<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n");
			config.append("<filterByColor>1</filterByColor>\n");
			config.append("<blobColor>");
			config.append((darkBlobs ? 0 : 255));
			config.append("</blobColor>\n");
			config.append("<filterByArea>1</filterByArea>\n");
			config.append("<minArea>");
			config.append(minArea);
			config.append("</minArea>\n");
			config.append("<maxArea>");
			config.append(Integer.MAX_VALUE);
			config.append("</maxArea>\n");
			config.append("<filterByCircularity>1</filterByCircularity>\n");
			config.append("<minCircularity>");
			config.append(circularity[0]);
			config.append("</minCircularity>\n");
			config.append("<maxCircularity>");
			config.append(circularity[1]);
			config.append("</maxCircularity>\n");
			config.append("<filterByInertia>1</filterByInertia>\n");
			config.append("<minInertiaRatio>0.1</minInertiaRatio>\n");
			config.append("<maxInertiaRatio>" + Integer.MAX_VALUE + "</maxInertiaRatio>\n");
			config.append("<filterByConvexity>1</filterByConvexity>\n");
			config.append("<minConvexity>0.95</minConvexity>\n");
			config.append("<maxConvexity>" + Integer.MAX_VALUE + "</maxConvexity>\n");
			config.append("</opencv_storage>\n");
			FileWriter writer;
			writer = new FileWriter(tempFile, false);
			writer.write(config.toString());
			writer.close();
			blobDet.read(tempFile.getPath());
		} catch (IOException e) {
			e.printStackTrace();
		}

		blobDet.detect(input, blobList);
	}




}
*/

