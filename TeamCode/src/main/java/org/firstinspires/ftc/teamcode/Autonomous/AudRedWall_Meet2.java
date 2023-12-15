package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class AudRedWall_Meet2 extends LinearOpMode {
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
        Pose2d RedPark = new Pose2d(13,45,Math.toRadians(90));
        Pose2d RedRallyPoint1_LEFT = new Pose2d(10.5,-50,Math.toRadians(90));//left and a bit forward from the left pixel drop
        Pose2d RedRallyPoint1_CENTER = new Pose2d(10.5,-40,Math.toRadians(90));//left and a bit forward from the center pixel drop
        Pose2d RedRallyPoint1_RIGHT = new Pose2d(10.5,-40,Math.toRadians(90));//left and a bit forward from the center pixel drop
        Pose2d RedRallyPoint2 = new Pose2d(10.5,-0,Math.toRadians(90));// near md field on red side


        //Center Prop
        Pose2d AudRedCenter = new Pose2d(28.5, -45, Math.toRadians(90));
        //backstage drop
        Pose2d AudRedCenterDropoff = new Pose2d(39, 51, Math.toRadians(90));

        // Left Prop Poses
        Pose2d AudRedLeft = new Pose2d(33.5,-57.5, Math.toRadians(90));
        Pose2d AudRedLeftDropoff = new Pose2d(29,51, Math.toRadians(90));

        // Right Prop Poses
        Pose2d AudRedRightDropoff = new Pose2d(44, 51, Math.toRadians(90));//backstage
        Pose2d AudRedRightPre = new Pose2d(35.5,-41, Math.toRadians(90));
        Pose2d AudRedRight = new Pose2d(35.5,-34,Math.toRadians(90));//spike mark



        drive.setPoseEstimate(startPos);

        //============================
        // TRAJECTORIES
        //============================

        //StageRedLeft
        TrajectorySequence StageRedLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5();})
                .lineToLinearHeading(AudRedLeft)
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperLeftOpen();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerCarry();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(RedRallyPoint1_LEFT)
                .lineToLinearHeading(RedRallyPoint2)
                .splineToLinearHeading(AudRedLeftDropoff,Math.toRadians(110))// was 0
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel2())
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(RedPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.125)
                .build();

        //StageRedCenter
        TrajectorySequence StageRedCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5();})
                .lineToLinearHeading(AudRedCenter)
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperLeftOpen();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerCarry();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(RedRallyPoint1_CENTER)
                .lineToLinearHeading(RedRallyPoint2)
                .splineToLinearHeading(AudRedCenterDropoff,Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel2())
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.125)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(RedPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.125)
                .build();


        //StageRedRight
        TrajectorySequence StageRedRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5();})
                .lineToLinearHeading(AudRedRightPre)
                .lineToLinearHeading(AudRedRight)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperLeftOpen();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerCarry();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(RedRallyPoint1_RIGHT)
                .lineToLinearHeading(RedRallyPoint2)
                .splineToLinearHeading(AudRedRightDropoff,Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel2())
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(RedPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.125)
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
