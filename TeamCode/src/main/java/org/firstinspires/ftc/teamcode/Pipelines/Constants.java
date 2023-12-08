package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class Constants {

    // STAGE SIDE CONSTANTS
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

    // AUDIENCE SIDE SPECIFIC CONSTANTS
    public static final Rect CENTER_ROI_BLUE_AUDIENCE = new Rect(
            new Point(180, 70),
            new Point(255, 140)
    );
    public static final Rect LEFT_ROI_BLUE_AUDIENCE = new Rect(
            new Point(0, 90),
            new Point(80, 175)
    );

    public static final Rect CENTER_ROI_RED_AUDIENCE = new Rect(
            new Point(55, 60),
            new Point(115, 130)
    );

    public static final Rect RIGHT_ROI_RED_AUDIENCE = new Rect(
            new Point(215, 80),
            new Point(280, 155)
    );
    public static final Scalar RED_LOW_HSV = new Scalar(0, 155,  115);
    public static final Scalar RED_HIGH_HSV = new Scalar(25, 255, 255);
    public static final Scalar BLUE_LOW_HSV = new Scalar(105, 85, 40);
    public static final Scalar BLUE_HIGH_HSV = new Scalar(115, 225, 225);
}
