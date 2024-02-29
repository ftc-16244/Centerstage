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
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class StageBlue20 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvWebcam webcam;
    int timesWebcamAttemptedOpen = 0;
    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern pipelineError = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineBroken = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineNotReady = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineReady = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
    private void openCamera() {
        timesWebcamAttemptedOpen++;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // this camera supports 1280x800, 1280x720, 800x600, 640x480, and 320x240
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE, OpenCvWebcam.StreamFormat.MJPEG);
            }
            @Override
            public void onError(int errorCode) {
                if(timesWebcamAttemptedOpen < 10) {
                    System.err.println("Webcam could not be opened! Retrying...");
                    openCamera();
                } else {
                    throw new OpenCvCameraException("Webcam failed to open 10 times!");
                }
            }
        });
    }
    @Override
    public void runOpMode() throws InterruptedException {

        Felipe2 felipe = new Felipe2(this);
        felipe.init(hardwareMap);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        blinkin.setPattern(pipelineNotReady);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Left"), cameraMonitorViewId);
        Pipeline detector = new Pipeline(telemetry, StartPosition.BLUE_STAGE, blinkin);
        webcam.setPipeline(detector);

        openCamera();

        //============================
        // ALL POSE SECTION
        //============================

        Pose2d startPos = new Pose2d(14.5,62,Math.toRadians(180));
        //Pose2d BlueCenterPark = new Pose2d(46,12,Math.toRadians(180));
        Pose2d BlueWallPark = new Pose2d(45,61,Math.toRadians(180));

        //============================
        // CENTER POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_CENTER = new Pose2d(53,35.5,Math.toRadians(180));
        Pose2d PurplePixelDropOff_CENTER = new Pose2d(22,21,Math.toRadians(180));

        //============================
        // RIGHT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_RIGHT = new Pose2d(52,28,Math.toRadians(180));
        Pose2d PurplePixelDropOff_RIGHT_1a = new Pose2d(18,31,Math.toRadians(180));
        Pose2d PurplePixelDropOff_RIGHT_1b = new Pose2d(3,33,Math.toRadians(180));
        Pose2d PurplePixelDropOff_RIGHT_1c = new Pose2d(12,33,Math.toRadians(180));

        //============================
        // LEFT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_LEFT = new Pose2d(55,44.5,Math.toRadians(180));
        Pose2d PurplePixelDropOff_LEFT_1a = new Pose2d(35.5,28,Math.toRadians(180));


        drive.setPoseEstimate(startPos);

        //============================
        // LEFT TRAJECTORY
        //============================
        TrajectorySequence StageBlueLeft = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                .waitSeconds(0.1)
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_LEFT_1a)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                //yellow pixel journey
                .lineToLinearHeading(YellowPixelDropOff_LEFT)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .forward(5)
                .lineToLinearHeading(BlueWallPark)
                .addTemporalMarker(()->felipe.setAnglerLoad())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperWideOpen();})
                .waitSeconds(1)
                .back(15)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperWideOpen();})
                .build();


        //============================
        // RIGHT TRAJECTORY
        //============================
        TrajectorySequence StageBlueRight = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                .waitSeconds(0.1)
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_RIGHT_1a)
                //.lineToLinearHeading(PurplePixelDropOff_RIGHT_1b)
                .lineToLinearHeading(PurplePixelDropOff_RIGHT_1c)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .waitSeconds(0.1)
                .back(10)
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                //yellow pixel journey
                .lineToLinearHeading(YellowPixelDropOff_RIGHT)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .forward(5)
                .lineToLinearHeading(BlueWallPark)
                .addTemporalMarker(()->felipe.setAnglerLoad())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperWideOpen();})
                .waitSeconds(1)
                .back(15)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperWideOpen();})
                .build();

        //============================
        // CENTER TRAJECTORY
        //============================
        TrajectorySequence StageBlueCenter = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                .waitSeconds(0.1)
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_CENTER)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .waitSeconds(0.1)
                .back(10)
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                //yellow pixel journey
                .lineToLinearHeading(YellowPixelDropOff_CENTER)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .forward(5)
                .lineToLinearHeading(BlueWallPark)
                .addTemporalMarker(()->felipe.setAnglerLoad())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperWideOpen();})
                .waitSeconds(1)
                .back(15)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperWideOpen();})
                .build();

        felipe.setAnglerLoad();
        sleep(250);
        felipe.gripperClosed();
        sleep(250);
        felipe.slideMechanicalReset();
        sleep(250);felipe.gripperWideOpen();
        sleep(250);

        System.out.println("Current location: " + detector.getPropLocation());

        waitForStart();


        detector.toggleTelemetry();
        telemetry.clearAll();

        int totalTimeWaited = 0;
        boolean pipelineRan = true;
        if(detector.getPropLocation() == null) {
            telemetry.addData("ERROR", "Start was pressed too soon.");
            telemetry.update();
            blinkin.setPattern(pipelineError);

            while(detector.getPropLocation() == null && totalTimeWaited < 3000 && !isStopRequested()) {
                totalTimeWaited += (webcam.getOverheadTimeMs() * 4);
                sleep(webcam.getOverheadTimeMs() * 4L);
            }
            telemetry.addData("Wasted time", totalTimeWaited);
            if(totalTimeWaited > 3000) {
                telemetry.addData("ERROR", "The pipeline never ran.");
                pipelineRan = false;
                blinkin.setPattern(pipelineBroken);
            }
            telemetry.update();
        }
        else {
            telemetry.addData("INFO", "Pipeline is running correctly");
            telemetry.update();
            blinkin.setPattern(pipelineReady);
        }

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        //detector.toggleTelemetry();
        //telemetry.clearAll();

        Prop location = Prop.CENTER;

        if(pipelineRan) {
            location = detector.getPropLocation();
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        telemetry.addData("Running path", " BLUE_STAGE_" + location);
        telemetry.update();

        switch(location) {
            case LEFT:
                drive.followTrajectorySequence(StageBlueLeft);
                break;
            case CENTER:
                drive.followTrajectorySequence(StageBlueCenter);
                break;
            case RIGHT:
                drive.followTrajectorySequence(StageBlueRight);
                break;
            default:
                drive.followTrajectorySequence(StageBlueCenter);
                break;
        }
    }
}