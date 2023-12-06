package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class Constants {
    public static final Rect LEFT_ROI_RED = new Rect(
            new Point(25, 70),
            new Point(100, 150)
    );
    public static final Rect CENTER_ROI_RED = new Rect(
            new Point(25, 60),
            new Point(85, 130)
    );
    public static final Rect RIGHT_ROI_RED = new Rect(
            new Point(170, 70),
            new Point(235, 145)
    );
    public static final Rect LEFT_ROI_BLUE = new Rect(
            new Point(60, 90),
            new Point(140, 175)
    );
    public static final Rect CENTER_ROI_BLUE = new Rect(
            new Point(230, 70),
            new Point(295, 140)
    );
    public static final Rect RIGHT_ROI_BLUE = new Rect(
            new Point(25, 70),
            new Point(100, 150)
    );
    public static final Scalar RED_LOW_HSV = new Scalar(0, 155,  115);
    public static final Scalar RED_HIGH_HSV = new Scalar(25, 255, 255);
    public static final Scalar BLUE_LOW_HSV = new Scalar(105, 85, 40);
    public static final Scalar BLUE_HIGH_HSV = new Scalar(115, 225, 225);
}
