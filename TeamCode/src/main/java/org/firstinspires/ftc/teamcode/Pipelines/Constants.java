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
            new Point(55, 40), //was 185,55
            new Point(110, 100) //was 255,125
    );
    public static final Rect RIGHT_ROI_RED = new Rect(
            new Point(140, 80),
            new Point(200, 150)
    );
    public static final Rect LEFT_ROI_BLUE = new Rect(
            new Point(50, 75),
            new Point(130, 145)
    );
    public static final Rect CENTER_ROI_BLUE = new Rect(
            new Point(225, 65),
            new Point(290, 130)
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
