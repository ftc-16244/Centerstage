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
public class StageRed22 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvWebcam webcam;
    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern pipelineError = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineBroken = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineNotReady = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
    RevBlinkinLedDriver.BlinkinPattern pipelineReady = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;

    @Override
    public void runOpMode() throws InterruptedException {

        Felipe2 felipe2 = new Felipe2(this);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        blinkin.setPattern(pipelineNotReady);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Blue"), cameraMonitorViewId);
        Pipeline detector = new Pipeline(telemetry, StartPosition.RED_STAGE, blinkin);
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
        felipe2.setAnglerLoad();
        sleep(250);
        felipe2.gripperWideOpen();
        felipe2.slideMechanicalReset();
        felipe2.setTurnerLoad();
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

        //============================
        // ALL POSE SECTION
        //============================

        Pose2d startPos = new Pose2d(10,-56,Math.toRadians(180));
        Pose2d wallpark = new Pose2d(1,1,Math.toRadians(180));

        //============================
        // CENTER POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_CENTER = new Pose2d(45,-37,Math.toRadians(180));
        Pose2d PurplePixelDropOff_CENTER = new Pose2d(22,-25,Math.toRadians(180));

        /**
         * the number 1 symbolizes the journey from backstage to audience
         * the number 2 symbolizes the journey from audience to backstage
         * letter symbolizes the different parts of the path
         */

        Pose2d WhiteTravelPart1a_CENTER = new Pose2d(-54,-36, Math.toRadians(180));

        Pose2d WhiteTravelPart2a_CENTER = new Pose2d(10,-35, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_Center = new Pose2d (48,-56, Math.toRadians(180));

        //============================
        // LEFT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_LEFT = new Pose2d(45,-29,Math.toRadians(180));
        Pose2d PurplePixelDropOff_LEFT_1a = new Pose2d(10,-35,Math.toRadians(180));
        Pose2d PurplePixelDropOff_LEFT_1b = new Pose2d(4,-35,Math.toRadians(180));

        /**
         * the number 1 symbolizes the journey from backstage to audience
         * the number 2 symbolizes the journey from audience to backstage
         * letter symbolizes the different parts of the path
         */

        Pose2d WhiteTravelPart1a_LEFT = new Pose2d(-0,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1b_LEFT = new Pose2d(-20,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1c_LEFT = new Pose2d(-42,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1e_LEFT = new Pose2d(-56,-36, Math.toRadians(180));

        Pose2d WhiteTravelPart2a_LEFT = new Pose2d(-42,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_LEFT = new Pose2d(-20,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart2c_LEFT = new Pose2d(0,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart2d_LEFT = new Pose2d(45,-60, Math.toRadians(180));

        //============================
        // RIGHT POSE SECTION
        //============================

        Pose2d YellowPixelDropOff_RIGHT = new Pose2d(0,0,Math.toRadians(180));
        Pose2d PurplePixelDropOff_RIGHT = new Pose2d(0,0,Math.toRadians(180));

        /**
         * the number 1 symbolizes the journey from backstage to audience
         * the number 2 symbolizes the journey from audience to backstage
         * letter symbolizes the different parts of the path
         */

        Pose2d WhiteTravelPart1a_RIGHT = new Pose2d(-0,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1b_RIGHT = new Pose2d(-20,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1c_RIGHT = new Pose2d(-42,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1e_RIGHT = new Pose2d(-56,-36, Math.toRadians(180));

        Pose2d WhiteTravelPart2a_RIGHT = new Pose2d(-42,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_RIGHT = new Pose2d(-20,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart2c_RIGHT = new Pose2d(0,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart2d_RIGHT = new Pose2d(45,-60, Math.toRadians(180));


        drive.setPoseEstimate(startPos);


        //============================
        // RIGHT TRAJECTORY
        //============================
        TrajectorySequence StageRedRight = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerAuto();})
                /**
                 * purple pixel journey
                 */
                //head over to purple pixel push prop and then come back
                //add lines to to do right purple pixel dropoff
                //open gripper
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperRightOpen();})
                //wait and back out
                .waitSeconds(0.125)
                .back(15)
                /**
                 * yellow pixel journey
                 */
                //initialize all the subsystems to get ready to drop the yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerDeploy();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideRow_1();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerDeploy();})
                //go to yellow pixel drop off
                //add line to linear heading to yellow pixel dropoff right
                .lineToLinearHeading(YellowPixelDropOff_RIGHT)
                .waitSeconds(0.125)
                //drop pixel and head back
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperLeftOpen();})
                .waitSeconds(0.125)
                .back(10)
                /**
                 * white pixel journey 1
                 */
                //set turner to load, set angler to auto, and slide level 0
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerLoad();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerLoad();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideLevel_0();})
                //spline to white pixel stacks
                .splineToLinearHeading(WhiteTravelPart1a_RIGHT, Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1b_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1c_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1e_RIGHT, Math.toRadians(180))
                //set turner to 11 degrees
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .waitSeconds(0.125)
                .back(5)
                //spline to backstage area
                .splineToLinearHeading(WhiteTravelPart2a_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2b_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2c_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2d_RIGHT,Math.toRadians(180))
                //set turner to deploy, set slide level to 1, open gripper
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerDeploy();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideRow_1();})
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperOpen();})
                .waitSeconds(0.125)
                /**
                 * white pixel journey 2
                 */
                //set turner to load again, set slide level 0
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerLoad();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideLevel_0();})
                //spline to white stacks again
                .splineToLinearHeading(WhiteTravelPart1a_RIGHT, Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1b_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1c_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1e_RIGHT, Math.toRadians(180))
                //set turner to 11 degrees
                //grip and back out
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .waitSeconds(0.125)
                .back(5)
                //spline to backstage area again
                .splineToLinearHeading(WhiteTravelPart2a_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2b_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2c_RIGHT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2d_RIGHT,Math.toRadians(180))
                //drop pixels, and back out
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperOpen();})
                .back(25)
                .build();


        //============================
        // LEFT TRAJECTORY
        //============================
        TrajectorySequence StageRedLeft = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerAuto();})
                /**
                 * purple pixel journey
                 */
                //head over to purple pixel push prop and then come back
                .lineToLinearHeading(PurplePixelDropOff_LEFT_1a)
                .lineToLinearHeading(PurplePixelDropOff_LEFT_1b)
                //open gripper
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperRightOpen();})
                //wait and back out
                .waitSeconds(0.125)
                .back(15)
                /**
                 * yellow pixel journey
                 */
                //initialize all the subsystems to get ready to drop the yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerDeploy();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideRow_1();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerDeploy();})
                //go to yellow pixel drop off
                .lineToLinearHeading(YellowPixelDropOff_LEFT)
                .waitSeconds(0.125)
                //drop pixel and head back
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperLeftOpen();})
                .waitSeconds(0.125)
                .back(10)
                /**
                 * white pixel journey 1
                 */
                //strafe left to avoid conflict
                .strafeLeft(15)
                //set turner to load, set angler to auto, and slide level 0
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerLoad();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerLoad();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideLevel_0();})
                //spline to white pixel stacks
                .splineToLinearHeading(WhiteTravelPart1a_LEFT, Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1b_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1c_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1e_LEFT, Math.toRadians(180))
                //set turner to 11 degrees
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .waitSeconds(0.125)
                .back(5)
                //spline to backstage area
                .splineToLinearHeading(WhiteTravelPart2a_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2b_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2c_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2d_LEFT,Math.toRadians(180))
                //set turner to deploy, set slide level to 1, open gripper
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerDeploy();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideRow_1();})
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperOpen();})
                .waitSeconds(0.125)
                /**
                 * white pixel journey 2
                 */
                //set turner to load again, set slide level 0
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerLoad();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideLevel_0();})
                //spline to white stacks again
                .splineToLinearHeading(WhiteTravelPart1a_LEFT, Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1b_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1c_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart1e_LEFT, Math.toRadians(180))
                //set turner to 11 degrees
                //grip and back out
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .waitSeconds(0.125)
                .back(5)
                //spline to backstage area again
                .splineToLinearHeading(WhiteTravelPart2a_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2b_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2c_LEFT,Math.toRadians(180))
                .splineToLinearHeading(WhiteTravelPart2d_LEFT,Math.toRadians(180))
                //drop pixels, and back out
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperOpen();})
                .back(25)
                .build();

        //============================
        // CENTER TRAJECTORY
        //============================
        TrajectorySequence StageRedCenter = drive.trajectorySequenceBuilder(startPos)
                //initialize all the subsystems and drive the purple pixel dropoff
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerAuto();})
                /**
                 * purple pixel journey
                 */
                .lineToLinearHeading(PurplePixelDropOff_CENTER)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperRightOpen();})
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerDeploy();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerDeploy();})
                //release right gripper, turn the turner, set the slide one, and angle the angler to deploy
                //go the the backstage, to the center position
                /**
                 * yellow pixel journey
                 */
                .lineToLinearHeading(YellowPixelDropOff_CENTER)
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperLeftOpen();})
                .waitSeconds(0.125)
                .back(12)
                /**
                 * white pixel journey 1
                 */
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerLoad();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setAnglerLoad();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideLevel_0();})
                //back up and put turner to load, angler to load, slide to level 0
                //go under the truss closest the the centerstage door
                .lineToLinearHeading(WhiteTravelPart1a_CENTER)
                //add a line that makes the turner turn 11 degrees
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .waitSeconds(0.125)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideLevel_0();})
                //set to slide level 1, and close gripper
                //back up and put slide level to 0
                //travel back to under the truss
                .lineToLinearHeading(WhiteTravelPart2a_CENTER)
                .strafeLeft(30)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerDeploy();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperOpen();})
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerLoad();})
                //turn the turner
                //travel to the backstage but make an L motion to avoid crashing but drop on ground
                //open gripper
                //turn the turner back
                //travel back under the truss
                .lineToLinearHeading(WhiteTravelPart2b_Center)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperOpen();})
                .waitSeconds(0.125)
                /**
                 * white pixel journey 2
                 */
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setTurnerLoad();})
                //go back to right before the truss location
                .lineToLinearHeading(WhiteTravelPart2a_CENTER)
                //go to white stacks and pick up 3rd and 4th white pixel
                .lineToLinearHeading(WhiteTravelPart1a_CENTER)
                //set the turner to 11 degrees
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperClosed();})
                .waitSeconds(0.125)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.setSlideLevel_0();})
                //set to slide level 1, and close gripper
                //back up and put slide level to 0
                //travel back under the truss
                .lineToLinearHeading(WhiteTravelPart2a_CENTER)
                .strafeLeft(20)
                //make an L motion and travel to backstage but drop on ground
                .lineToLinearHeading(WhiteTravelPart2b_Center)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{felipe2.gripperOpen();})
                .back(25)
                //drop pixels and back up and park
                .build();


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
    }
}