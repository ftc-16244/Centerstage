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
//@Disabled
public class AudRed20 extends LinearOpMode {
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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Red"), cameraMonitorViewId);
        Pipeline detector = new Pipeline(telemetry, StartPosition.RED_AUD, blinkin);
        webcam.setPipeline(detector);

        openCamera();

        //============================
        // ALL POSE SECTION
        //============================

        Pose2d startPos = new Pose2d(-34,-64,Math.toRadians(0));

        Pose2d Splinept1 = new Pose2d(-34,-11,Math.toRadians(0));
        Pose2d Splinept2 = new Pose2d(33,-11,Math.toRadians(0));


        //============================
        // CENTER POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_CENTER = new Pose2d(53.5,-32,Math.toRadians(180));
        Pose2d PurplePixelDropOff_CENTER = new Pose2d(-41,-22,Math.toRadians(0));


        //============================
        // LEFT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_LEFT = new Pose2d(51,-29,Math.toRadians(180));
        Pose2d PurplePixelDropOff_LEFT_1a = new Pose2d(-54.5,-29,Math.toRadians(0));


        //============================
        // RIGHT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_RIGHT = new Pose2d(52,-41,Math.toRadians(180));
        Pose2d PurplePixelDropOff_RIGHT_1a = new Pose2d(-37,-32,Math.toRadians(0));
        Pose2d PurplePixelDropOff_RIGHT_1b = new Pose2d(-31,-32,Math.toRadians(0));



        drive.setPoseEstimate(startPos);


        //============================
        // RIGHT TRAJECTORY
        //============================
        TrajectorySequence AudRedRight = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_RIGHT_1a)
                .lineToLinearHeading(PurplePixelDropOff_RIGHT_1b)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .addTemporalMarker(()->felipe.setAnglerAuto())
                .back(15)
                .strafeLeft(10)
                //yellow pixel journey
                .splineToLinearHeading(Splinept1,Math.toRadians(0))
                .splineToLinearHeading(Splinept2,Math.toRadians(0))
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(YellowPixelDropOff_RIGHT,Math.toRadians(180))
                .back(5)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .waitSeconds(0.1)
                .forward(5)
                .strafeRight(30)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .back(11)
                .build();


        //============================
        // LEFT TRAJECTORY
        //============================
        TrajectorySequence AudRedLeft = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_LEFT_1a)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->felipe.setAnglerAuto())
                .strafeLeft(10)
                //yellow pixel journey
                .splineToLinearHeading(Splinept1,Math.toRadians(0))
                .splineToLinearHeading(Splinept2,Math.toRadians(0))
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(YellowPixelDropOff_LEFT,Math.toRadians(180))
                .back(5)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .waitSeconds(0.1)
                .forward(5)
                .strafeRight(15)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .back(11)
                .build();

        //============================
        // CENTER TRAJECTORY
        //============================
        TrajectorySequence AudRedCenter = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_CENTER)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .waitSeconds(0.5)
                .back(10)
                .addTemporalMarker(()->felipe.setAnglerAuto())
                .strafeLeft(10)
                //yellow pixel journey
                .splineToLinearHeading(Splinept1,Math.toRadians(0))
                .splineToLinearHeading(Splinept2,Math.toRadians(0))
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(YellowPixelDropOff_CENTER,Math.toRadians(180))
                .forward(5)
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .waitSeconds(0.1)
                .forward(5)
                .strafeRight(24)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .back(16)
                .build();

        felipe.setAnglerLoad();
        sleep(250);
        felipe.gripperClosed();
        sleep(250);
        felipe.slideMechanicalReset();
        sleep(250);

        waitForStart();


        detector.toggleTelemetry();
        telemetry.clearAll();

        int totalTimeWaited = 0;
        boolean pipelineRan = true;
        if(detector.getPropLocation() == null) {
            telemetry.addData("ERROR", "Start was pressed too soon.");
            telemetry.update();
            blinkin.setPattern(pipelineError);

            while(detector.getPropLocation() == null && totalTimeWaited < 7000 && !isStopRequested()) {
                totalTimeWaited += (webcam.getOverheadTimeMs() * 4);
                sleep(webcam.getOverheadTimeMs() * 4L);
            }
            telemetry.addData("Wasted time", totalTimeWaited);
            if(totalTimeWaited > 7000) {
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

        telemetry.addData("Running path", " RED_STAGE" + location);
        telemetry.update();

        drive.followTrajectorySequence(AudRedRight);


        /*
        switch(location) {
            case LEFT:
                drive.followTrajectorySequence(AudRedLeft);
                break;
            case CENTER:
                drive.followTrajectorySequence(AudRedCenter);
                break;
            case RIGHT:
                drive.followTrajectorySequence(AudRedRight);
                break;
            default:
                throw new IllegalArgumentException("the code did not detect the prop at all, and it is running the default case.");
        }

         */
    }
}