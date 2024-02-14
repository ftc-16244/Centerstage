package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class Constants {
    public static final Rect LEFT_ROI_RED = new Rect(
            new Point(40, 130),
            new Point(140, 245)
    );
    public static final Rect CENTER_ROI_RED = new Rect(
            new Point(290, 110),
            new Point(390, 210)
    );
    public static final Rect RIGHT_ROI_RED = new Rect(
            new Point(500, 130),
            new Point(610, 235)
    );
    public static final Rect LEFT_ROI_BLUE = new Rect(
            new Point(40, 75),
            new Point(120, 145)
    );
    public static final Rect CENTER_ROI_BLUE = new Rect(
            new Point(215, 65),
            new Point(270, 125)
    );
    public static final Rect RIGHT_ROI_BLUE = new Rect(
            new Point(25, 70),
            new Point(100, 150)
    );
    public static final Scalar RED_LOW_HSV = new Scalar(0, 155,  115);
    public static final Scalar RED_HIGH_HSV = new Scalar(25, 255, 255);
    public static final Scalar BLUE_LOW_HSV = new Scalar(95, 100, 15);
    public static final Scalar BLUE_HIGH_HSV = new Scalar(125, 255, 255);
}
