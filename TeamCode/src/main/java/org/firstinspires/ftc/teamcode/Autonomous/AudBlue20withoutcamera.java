package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsytems.Felipe2;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.Pipeline;
import org.firstinspires.ftc.teamcode.Pipelines.Prop;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
//@Disabled
public class AudBlue20withoutcamera extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Felipe2 felipe = new Felipe2(this);
        felipe.init(hardwareMap);

        //============================
        // ALL POSE SECTION
        //============================

        Pose2d startPos = new Pose2d(-34,64,Math.toRadians(0));

        Pose2d Splinept1 = new Pose2d(-35,12,Math.toRadians(180));
        Pose2d Splinept2 = new Pose2d(32,13,Math.toRadians(180));



        //============================
        // CENTER POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_CENTER = new Pose2d(51.5,39.5,Math.toRadians(180));
        Pose2d PurplePixelDropOff_CENTER = new Pose2d(-45,27,Math.toRadians(0));


        //============================
        // RIGHT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_Right = new Pose2d(51,32,Math.toRadians(180));
        Pose2d PurplePixelDropOff_Right_1a = new Pose2d(-54,32,Math.toRadians(0));


        //============================
        // LEFT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_LEFT = new Pose2d(52,41,Math.toRadians(180));
        Pose2d PurplePixelDropOff_LEFT_1a = new Pose2d(-37,34,Math.toRadians(0));
        Pose2d PurplePixelDropOff_LEFT_1b = new Pose2d(-32,32,Math.toRadians(0));

        drive.setPoseEstimate(startPos);

        /*
        //============================
        // LEFT TRAJECTORY
        //============================
        TrajectorySequence AudBlueLeft = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_LEFT_1a)
                .lineToLinearHeading(PurplePixelDropOff_LEFT_1b)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .addTemporalMarker(()->felipe.setAnglerAuto())
                .back(15)
                .strafeRight(25)
                //yellow pixel journey
                .splineToLinearHeading(Splinept1,Math.toRadians(180))
                .splineToLinearHeading(Splinept2,Math.toRadians(180))
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(YellowPixelDropOff_LEFT,Math.toRadians(180))
                .back(5)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .waitSeconds(0.1)
                .forward(5)
                .strafeLeft(30)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .back(11)
                .build();


        //============================
        // RIGHT TRAJECTORY
        //============================
        TrajectorySequence AudBlueRight = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_Right_1a)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->felipe.setAnglerAuto())
                .strafeRight(20)
                //yellow pixel journey
                .splineToLinearHeading(Splinept1,Math.toRadians(180))
                .splineToLinearHeading(Splinept2,Math.toRadians(180))
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(YellowPixelDropOff_Right,Math.toRadians(180))
                .back(5)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .waitSeconds(0.1)
                .forward(5)
                .strafeLeft(21)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .back(11)
                .build();


         */
        //============================
        // CENTER TRAJECTORY
        //============================

        felipe.setAnglerLoad();
        sleep(250);
        felipe.gripperClosed();
        sleep(250);
        felipe.slideMechanicalReset();
        sleep(250);

        waitForStart();

        //detector.toggleTelemetry();
        //telemetry.clearAll();


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        TrajectorySequence AudBlueCenter = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_CENTER)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(()->felipe.setAnglerAuto())
                .strafeRight(10)
                //yellow pixel journey
                .splineToLinearHeading(Splinept1,Math.toRadians(180))
                .splineToLinearHeading(Splinept2,Math.toRadians(180))
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(YellowPixelDropOff_CENTER,Math.toRadians(180))
                .back(2)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .waitSeconds(0.1)
                .forward(5)
                .strafeLeft(24)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .back(12)
                .build();



        //detector.toggleTelemetry();
        //telemetry.clearAll();

        drive.followTrajectorySequence(AudBlueCenter);


        /*
        switch(location) {
            case LEFT:
                drive.followTrajectorySequence(StageRedLeft);
                break;
            case CENTER:
                drive.followTrajectorySequence(StageRedCenter);
                break;
            case RIGHT:
                drive.followTrajectorySequence(StageRedRight);
                break;
            default:
                throw new IllegalArgumentException("the code did not detect the prop at all, and it is running the default case.");
        }

         */
    }
}