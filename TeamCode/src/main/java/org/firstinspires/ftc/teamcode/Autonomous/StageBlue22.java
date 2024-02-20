package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

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
@Disabled
public class StageBlue22 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvWebcam webcam;
    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern pipelineError = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineBroken = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineNotReady = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineReady = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;

    @Override
    public void runOpMode() throws InterruptedException {

        Felipe2 felipe = new Felipe2(this);
        felipe.init(hardwareMap);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        blinkin.setPattern(pipelineNotReady);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Blue"), cameraMonitorViewId);
        Pipeline detector = new Pipeline(telemetry, StartPosition.BLUE_STAGE, blinkin);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // this camera supports 1280x800, 1280x720, 800x600, 640x480, and 320x240
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SENSOR_NATIVE, OpenCvWebcam.StreamFormat.MJPEG);
            }
            @Override
            public void onError(int errorCode) {}
        });

        //============================
        // ALL POSE SECTION
        //============================

        Pose2d startPos = new Pose2d(14.5,62,Math.toRadians(180));
        Pose2d wallpark = new Pose2d(1,1,Math.toRadians(180));

        //============================
        // CENTER POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_CENTER = new Pose2d(53,35.5,Math.toRadians(180));
        Pose2d PurplePixelDropOff_CENTER = new Pose2d(22,21,Math.toRadians(180));

        /**
         * the number 1 symbolizes the journey from backstage to audience
         * the number 2 symbolizes the journey from audience to backstage
         * letter symbolizes the different parts of the path
         */

        Pose2d WhiteTravelPart1a_CENTER_pt1 = new Pose2d(-54,36, Math.toRadians(180));

        //Pose2d WhiteTravelPart2a_CENTER = new Pose2d(10,-35, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_Center = new Pose2d (48,56, Math.toRadians(180));

        Pose2d WhiteTravelPart1a_CENTER = new Pose2d(-0,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1b_CENTER = new Pose2d(-20,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1c_CENTER = new Pose2d(-42,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1e_CENTER = new Pose2d(-56,36, Math.toRadians(180));

        Pose2d WhiteTravelPart2a_CENTER = new Pose2d(-42,56, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_CENTER = new Pose2d(-20,56, Math.toRadians(180));
        Pose2d WhiteTravelPart2c_CENTER = new Pose2d(0,60, Math.toRadians(180));
        Pose2d WhiteTravelPart2d_CENTER = new Pose2d(28,60, Math.toRadians(180));
        Pose2d WhiteTravelPart2e_CENTER = new Pose2d(45,60, Math.toRadians(180));

        //============================
        // RIGHT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_RIGHT = new Pose2d(52,26,Math.toRadians(180));
        Pose2d PurplePixelDropOff_RIGHT_1a = new Pose2d(18,33,Math.toRadians(180));
        Pose2d PurplePixelDropOff_RIGHT_1b = new Pose2d(3,33,Math.toRadians(180));
        Pose2d PurplePixelDropOff_RIGHT_1c = new Pose2d(12,33,Math.toRadians(180));


        /**
         * the number 1 symbolizes the journey from backstage to audience
         * the number 2 symbolizes the journey from audience to backstage
         * letter symbolizes the different parts of the path
         */

        Pose2d WhiteTravelPart1a_RIGHT = new Pose2d(-0,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1b_RIGHT = new Pose2d(-20,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1c_RIGHT = new Pose2d(-42,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1e_RIGHT = new Pose2d(-56,36, Math.toRadians(180));

        Pose2d WhiteTravelPart2a_RIGHt = new Pose2d(-42,60, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_RIGHT = new Pose2d(-20,60, Math.toRadians(180));
        Pose2d WhiteTravelPart2c_RIGHT = new Pose2d(0,60, Math.toRadians(180));
        Pose2d WhiteTravelPart2d_RIGHT = new Pose2d(45,60, Math.toRadians(180));

        //============================
        // LEFT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_LEFT = new Pose2d(55,44.5,Math.toRadians(180));
        Pose2d PurplePixelDropOff_LEFT_1a = new Pose2d(34,28,Math.toRadians(180));

        /**
         * the number 1 symbolizes the journey from backstage to audience
         * the number 2 symbolizes the journey from audience to backstage
         * letter symbolizes the different parts of the path
         */

        Pose2d WhiteTravelPart1a_LEFT = new Pose2d(-0,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1b_LEFT = new Pose2d(-20,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1c_LEFT = new Pose2d(-42,60, Math.toRadians(180));
        Pose2d WhiteTravelPart1e_LEFT = new Pose2d(-56,36, Math.toRadians(180));

        Pose2d WhiteTravelPart2a_LEFT = new Pose2d(-42,60, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_LEFT = new Pose2d(-20,60, Math.toRadians(180));
        Pose2d WhiteTravelPart2c_LEFT = new Pose2d(0,60, Math.toRadians(180));
        Pose2d WhiteTravelPart2d_LEFT = new Pose2d(46,60, Math.toRadians(180));


        drive.setPoseEstimate(startPos);


        //============================
        // LEFT TRAJECTORY
        //============================
        TrajectorySequence StageRedLeft = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_LEFT_1a)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperRightOpen();})
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(()-> felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()-> felipe.setSlideRow_1())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .lineToLinearHeading(YellowPixelDropOff_LEFT)
                //release right gripper, turn the turner, set the slide one, and angle the angler to deploy
                //go the the backstage, to the center position
                //yellow pixel journey
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .forward(5)
                .strafeRight(20)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerLoad();})
                //spline to white pixel stacks
                .splineToLinearHeading(WhiteTravelPart1a_LEFT, Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1b_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1c_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1e_LEFT, Math.toRadians(180))
                //set turner to 11 degrees
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .waitSeconds(0.125)
                .back(5)
                //spline to backstage area
                .splineToLinearHeading(WhiteTravelPart2a_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2d_LEFT,Math.toRadians(180))
                .addTemporalMarker(()->felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(WhiteTravelPart2e_CENTER,Math.toRadians(180))
                .addTemporalMarker(()->felipe.gripperOpen())
                .waitSeconds(0.5)
                .addTemporalMarker(()->felipe.setTurnerAutoLOAD())
                .build();


        //============================
        // RIGHT TRAJECTORY
        //============================
        TrajectorySequence StageRedRight = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
                //purple pixel journey
                .lineToLinearHeading(PurplePixelDropOff_RIGHT_1a)
                .lineToLinearHeading(PurplePixelDropOff_RIGHT_1b)
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
                .strafeRight(38)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                //spline to white pixel stacks
                .splineToLinearHeading(WhiteTravelPart1a_LEFT, Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1b_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1c_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1e_LEFT, Math.toRadians(180))
                //set turner to 11 degrees
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .waitSeconds(0.125)
                .back(5)
                //spline to backstage area
                .splineToLinearHeading(WhiteTravelPart2a_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2d_LEFT,Math.toRadians(180))
                .addTemporalMarker(()->felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(WhiteTravelPart2e_CENTER,Math.toRadians(180))
                .addTemporalMarker(()->felipe.gripperOpen())
                .waitSeconds(0.5)
                .addTemporalMarker(()->felipe.setTurnerAutoLOAD())
                .build();

        //============================
        // CENTER TRAJECTORY
        //============================
        TrajectorySequence StageRedCenter = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setAnglerAuto();})
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
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.setTurnerAutoLOAD();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperLeftOpen();})
                .forward(5)
                .strafeRight(30)
                //white pixel journey 1
                .addTemporalMarker(()->felipe.setTurnerAutoLOAD())
                .lineToLinearHeading(WhiteTravelPart1a_CENTER_pt1)
                //add a line that makes the turner turn 11 degrees
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe.gripperClosed();})
                .waitSeconds(0.5)
                //travel back to under the truss
                .splineToLinearHeading(WhiteTravelPart2a_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2d_LEFT,Math.toRadians(180))
                .addTemporalMarker(()->felipe.setTurnerAutoDEPLOY())
                .addTemporalMarker(()->felipe.setAnglerDeploy())
                .splineToLinearHeading(WhiteTravelPart2e_CENTER,Math.toRadians(180))
                .addTemporalMarker(()->felipe.gripperOpen())
                .waitSeconds(0.5)
                .addTemporalMarker(()->felipe.setTurnerAutoLOAD())
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

            while(detector.getPropLocation() == null && totalTimeWaited < 7000) {
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

        telemetry.addData("Running path", " BLUE_STAGE" + location);
        telemetry.update();

        drive.followTrajectorySequence(StageRedLeft);

        /*switch(location) {
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