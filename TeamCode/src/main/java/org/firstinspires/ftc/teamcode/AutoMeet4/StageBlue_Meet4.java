package org.firstinspires.ftc.teamcode.AutoMeet4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.Pipeline;
import org.firstinspires.ftc.teamcode.Pipelines.Prop;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class StageBlue_Meet4 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pipelineError = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineNotReady = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(this);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(pipelineNotReady);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Pipeline detector = new Pipeline(telemetry, StartPosition.BLUE_STAGE, blinkinLedDriver);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        // Initialize the sub systems. Note the init method is inside the subsystem class
        lift.init(hardwareMap);
        //sleep(500);

        // Set start positions
        lift.setAnglerLoad();
        sleep(250);
        lift.gripperWideOpen();
        lift.slideMechanicalReset();
        sleep(250); // no sleepy no workie. Need this to let the anger servo have time to move

        waitForStart();
        lift.gripperClosed();


        detector.toggleTelemetry();
        telemetry.clearAll();

        int totalTimeWaited = 0;
        boolean pipelineRan = true;
        if(detector.getPropLocation() == null) {
            blinkinLedDriver.setPattern(pipelineError);
            telemetry.addData("ERROR", "Start was pressed too soon.");
            telemetry.update();

            while(detector.getPropLocation() == null && totalTimeWaited < 7000) {
                totalTimeWaited += (webcam.getOverheadTimeMs() * 4);
                sleep(webcam.getOverheadTimeMs() * 4L);
            }
            telemetry.addData("Wasted time", totalTimeWaited);
            if(totalTimeWaited > 7000) {
                telemetry.addData("ERROR", "The pipeline never ran.");
                pipelineRan = false;
            }
            telemetry.update();
        }
        else {
            telemetry.addData("INFO", "Pipeline is running correctly");
            telemetry.update();
        }

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //============================
        // POSE SECTION
        //============================
        // Common poses for all 3 Red Stage Prop Positions
        Pose2d startPos = new Pose2d(-62.5, 12, Math.toRadians(90));
        Pose2d BlueWallPark = new Pose2d(-62,54,Math.toRadians(270));
        Pose2d BlueMidPark = new Pose2d(-8,50,Math.toRadians(90));

        Pose2d StageBlueLeft = new Pose2d(-45,22,Math.toRadians(0));//spike mark
        Pose2d StageBlueLeftDropoff = new Pose2d(-40.5, 53, Math.toRadians(90));//backstage

        Pose2d StageBlueCenter = new Pose2d(-38, 14, Math.toRadians(0));
        Pose2d BluePreParkCenter = new Pose2d(-59, 30, Math.toRadians(270));
        Pose2d StageBlueCenterDropoff = new Pose2d(-37.5, 54, Math.toRadians(90));

        Pose2d StageBlueRight = new Pose2d(-35,15.5, Math.toRadians(270));
        Pose2d StageBlueRightPush = new Pose2d(-35,10, Math.toRadians(270));
        Pose2d StageBlueRightDropoff = new Pose2d(-30,54.25, Math.toRadians(90));

        drive.setPoseEstimate(startPos);

        //============================
        // TRAJECTORIES
        //============================

        //StageRedLeft
        TrajectorySequence StageBlueLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .lineToLinearHeading(StageBlueLeftDropoff)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(1)
                .lineToLinearHeading(StageBlueLeft)
                .forward(8)
                .lineToLinearHeading(StageBlueLeft)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerLoad();})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .back(1.5)
                .lineToLinearHeading(BlueWallPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                .waitSeconds(1.0)
                .build();



        //StageRedCenter
        TrajectorySequence StageBlueCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerDeploy())
                .lineToLinearHeading(StageBlueCenterDropoff)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(1)
                .back(5)
                .lineToLinearHeading(StageBlueCenter)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .back(5)
                .lineToLinearHeading(BluePreParkCenter)
                .waitSeconds(0.125)
                .lineToLinearHeading(BlueWallPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(.5)
                .build();

        //StageRedRight
        TrajectorySequence StageBlueRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .lineToLinearHeading(StageBlueRightDropoff)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(1)
                .back(5)
                .waitSeconds(0.125)
                .lineToLinearHeading(StageBlueRight)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1point5();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerLoad();})
                .lineToLinearHeading(StageBlueRightPush)
                .lineToLinearHeading(StageBlueRight)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(.25)
                .lineToLinearHeading(BlueWallPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .waitSeconds(0.5)
                .build();



        detector.toggleTelemetry();
        telemetry.clearAll();

        Prop location = Prop.CENTER;

        if(pipelineRan) {
            location = detector.getPropLocation();
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        telemetry.addData("Running path", " BLUE_STAGE" + location);
        telemetry.update();

        switch(location) {
            case LEFT:
                drive.followTrajectorySequence(StageBlueLeftTraj1);
                break;
            case CENTER:
                drive.followTrajectorySequence(StageBlueCenterTraj1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(StageBlueRightTraj1);
                break;
            default:
                throw new IllegalArgumentException("The code is most certainly severely screwed up.");
        }
    }
}
