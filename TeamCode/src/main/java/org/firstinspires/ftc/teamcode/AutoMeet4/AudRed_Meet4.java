package org.firstinspires.ftc.teamcode.AutoMeet4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.Prop;
import org.firstinspires.ftc.teamcode.Pipelines.StartPosition;
import org.firstinspires.ftc.teamcode.Pipelines.WebcamPipeline;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class AudRed_Meet4 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(this);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        WebcamPipeline detector = new WebcamPipeline(telemetry, StartPosition.RED_AUD);
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
        Pose2d startPos = new Pose2d(64.5, -36, Math.toRadians(90));
        Pose2d RedPrePark = new Pose2d(13,45,Math.toRadians(90));
        Pose2d RedPark = new Pose2d(22,49,Math.toRadians(90));
        Pose2d RedRallyPoint1_LEFT = new Pose2d(10.5,-55,Math.toRadians(90));//left and a bit forward from the left pixel drop
        Pose2d RedRallyPoint1_CENTER = new Pose2d(10,-40,Math.toRadians(90));//left and a bit forward from the center pixel drop
        Pose2d RedRallyPoint1_RIGHT = new Pose2d(10.5,-40,Math.toRadians(90));//left and a bit forward from the center pixel drop
        Pose2d RedRallyPoint2 = new Pose2d(10.5,-0,Math.toRadians(90));// near md field on red side


        //Center Prop
        Pose2d AudRedCenter = new Pose2d(29.5, -45.5, Math.toRadians(90));
        Pose2d AudRedCenterPush = new Pose2d(22.5, -45.5, Math.toRadians(90));
        //backstage drop
        Pose2d AudRedCenterDropoff = new Pose2d(39.5, 46, Math.toRadians(90)); // was 51

        // Left Prop Poses
        Pose2d AudRedLeft = new Pose2d(25,-31, Math.toRadians(270)); // y was 57.5
        Pose2d AudRedLeftPush = new Pose2d(15,-40, Math.toRadians(270)); // y was 57.5
        Pose2d AudRedLeftDropoff = new Pose2d(31.25,52.5, Math.toRadians(90));

        // Right Prop Poses
        Pose2d AudRedRightDropoff = new Pose2d(43, 52, Math.toRadians(90));//backstage
        Pose2d AudRedRightPre = new Pose2d(35.5,-41, Math.toRadians(90));
        Pose2d AudRedRightPush = new Pose2d(35.5,-35,Math.toRadians(90));//spike mark
        Pose2d AudRedRight = new Pose2d(35.5,-36,Math.toRadians(90));//spike mark

        //white spike mark
        Pose2d leftwhitespikemark_RED_LEFT = new Pose2d(7.25, -48.5, Math.toRadians(270));
        Pose2d leftwhitespikemark_RED_CENTER = new Pose2d(10.25, -47.5, Math.toRadians(270)); //was 12
        Pose2d leftwhitespikemark_RED_RIGHT = new Pose2d(9.75, -47.5, Math.toRadians(270));


        drive.setPoseEstimate(startPos);

        //============================
        // TRAJECTORIES
        //============================

        //StageRedLeft
        TrajectorySequence StageRedLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                //set the gripper to close to pixels and move the lift and angler to deploy right
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5_white_RED();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerLoad())
                .back(10)
                //go to push the pixel away and then go to left position and open the left gripper
                .lineToLinearHeading(AudRedLeftPush)
                .lineToLinearHeading(AudRedLeft)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                //head back to avoid hitting pixels and then spline to white pixel spike mark
                .strafeRight(13)
                .lineToLinearHeading(leftwhitespikemark_RED_LEFT)
                //pick up the white pixel and then go back
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperClosed())
                .waitSeconds(0.25)
                .back(20)
                //set slide to 1 and put angler in carry position
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                //spline to backboard
                .lineToLinearHeading(RedRallyPoint1_LEFT)
                .splineToLinearHeading(RedRallyPoint2, Math.toRadians(90))
                .forward(24)
                .splineToLinearHeading(AudRedLeftDropoff,Math.toRadians(90))
                //set slide to 2 and open right gripper
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1point5_back())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.25)
                //go back 6 in and then set slide level to 1
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                //head over to park
                .back(3)
                .strafeLeft(4)
                .build();

        //StageRedCenter
        TrajectorySequence StageRedCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                //set the gripper to close to pixels and move the lift and angler to deploy right
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5_white_RED();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerLoad())
                //go to push the pixel away and then go to center position and open the left gripper
                .lineToLinearHeading(AudRedCenterPush)
                .lineToLinearHeading(AudRedCenter)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                //head back to avoid hitting pixels and then spline to white pixel spike mark
                .back(12)
                .lineToLinearHeading(leftwhitespikemark_RED_CENTER)
                //pick up the white pixel and then go back
                .forward(7.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperClosed())
                .waitSeconds(0.25)
                .back(10)
                //set slide to 1 and put angler in carry position
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                //spline to backboard
                .splineToLinearHeading(RedRallyPoint1_CENTER, Math.toRadians(90))
                .splineToLinearHeading(RedRallyPoint2, Math.toRadians(90))
                .forward(24)
                .splineToLinearHeading(AudRedCenterDropoff,Math.toRadians(90))
                //set slide to 2 and open right gripper
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1point5_back())
                .waitSeconds(1)
                .forward(6.25)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                //go back 6 in and then set slide level to 1
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                //head over to park
                .lineToLinearHeading(RedPark)
                .build();


        //StageRedRight
        TrajectorySequence StageRedRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                //set the gripper to close to pixels and move the lift and angler to deploy right
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5_white();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerLoad())
                //go to push the pixel away and then go to right position and open the left gripper
                .lineToLinearHeading(AudRedRightPre)
                .lineToLinearHeading(AudRedRightPush)
                .lineToLinearHeading(AudRedRight)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.25)
                //head back to avoid hitting pixels and then spline to white pixel spike mark
                .back(15)
                .splineToLinearHeading(leftwhitespikemark_RED_RIGHT, Math.toRadians(270))
                //pick up the white pixel and then go back
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperClosed())
                .waitSeconds(0.25)
                .back(10)
                //set slide to 1 and put angler in carry position
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerCarry())
                //spline to backboard
                .splineToLinearHeading(RedRallyPoint1_RIGHT, Math.toRadians(90))
                .splineToLinearHeading(RedRallyPoint2, Math.toRadians(90))
                .forward(24)
                .splineToLinearHeading(AudRedRightDropoff,Math.toRadians(90))
                //set slide to 2 and open right gripper
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1point5_back())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                //go back 6 in and then set slide level to 1
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                //head over to park
                .strafeLeft(4)
                .build();

        detector.toggleTelemetry();
        telemetry.clearAll();

        Prop location = Prop.CENTER;

        if(pipelineRan) {
            location = detector.getPropLocation();
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        telemetry.addData("Running path", " RED_AUD" + location);
        telemetry.update();

        switch(location) {
            case LEFT:
                drive.followTrajectorySequence(StageRedLeftTraj1);
                break;
            case CENTER:
                drive.followTrajectorySequence(StageRedCenterTraj1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(StageRedRightTraj1);
                break;
            default:
                throw new IllegalArgumentException("The code is most certainly severely screwed up.");
        }
    }
}
