package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class ConstantsForPositions {
    public static final Rect LEFT_ROI_RED = new Rect(
            new Point(25, 70),
            new Point(100, 150)
    );
    public static final Rect CENTER_ROI_RED = new Rect(
            new Point(185, 55),
            new Point(255, 125)
    );
    public static final Rect RIGHT_ROI_RED = new Rect(
            new Point(10, 70),
            new Point(90, 150)
    );
    public static final Rect LEFT_ROI_BLUE = new Rect(
            new Point(240, 75),
            new Point(310, 155)
    );
    public static final Rect CENTER_ROI_BLUE = new Rect(
            new Point(105, 65),
            new Point(165, 130)
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
