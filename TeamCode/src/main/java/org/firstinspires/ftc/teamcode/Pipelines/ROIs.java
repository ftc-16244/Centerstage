package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Point;
import org.opencv.core.Rect;

public class ROIs {
    public static Rect LEFT_ROI_RED = new Rect(
            new Point(25, 70),
            new Point(100, 150)
    );
    public static Rect CENTER_ROI_RED = new Rect(
            new Point(185, 55),
            new Point(255, 125)
    );
    public static Rect RIGHT_ROI_RED = new Rect(
            new Point(10, 70),
            new Point(90, 150)
    );
    public static Rect LEFT_ROI_BLUE = new Rect(
            new Point(),
            new Point()
    );
    public static Rect CENTER_ROI_BLUE = new Rect(
            new Point(),
            new Point()
    );
    public static Rect RIGHT_ROI_BLUE = new Rect(
            new Point(),
            new Point()
    );
}
