package org.firstinspires.ftc.teamcode.AutoMeet3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class StageRed_4Pixels extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(this);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        WebcamPipeline detector = new WebcamPipeline(telemetry, StartPosition.RED_STAGE);
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
        Pose2d startPos = new Pose2d(62.5, 12, Math.toRadians(90));
        //Pose2d RedPark = new Pose2d(64,50,Math.toRadians(90));
        Pose2d RedWallPark = new Pose2d(60,55,Math.toRadians(270));

        //Center Prop
        Pose2d StageRedCenter = new Pose2d(38, 14, Math.toRadians(180));
        //backstage drop
        Pose2d StageRedCenterDropoff = new Pose2d(32, 52.5, Math.toRadians(90));
        Pose2d StageRedCenterDropoff2 = new Pose2d(32, 50, Math.toRadians(90));


        // Left Prop Poses
        Pose2d StageRedLeft = new Pose2d(30,19, Math.toRadians(270));
        Pose2d StageRedLeftDropoff = new Pose2d(26,54, Math.toRadians(90));

        // Right Prop Poses
        Pose2d StageRedRight = new Pose2d(36,12.5,Math.toRadians(90));//spike mark
        Pose2d StageRedRightDropoff = new Pose2d(39, 53, Math.toRadians(90));//backstage

        Pose2d StageSplineMidPoint1 = new Pose2d(56,18, Math.toRadians(270));
        Pose2d StageSplineMidPoint2 = new Pose2d(56,18, Math.toRadians(270));
        Pose2d RallyPoint_whitespikemark1 = new Pose2d(54, -28, Math.toRadians(270));
        Pose2d RallyPoint_whitespikemark2 = new Pose2d(53, -31, Math.toRadians(270));

        Pose2d midwhitespikemark = new Pose2d(20, -51, Math.toRadians(270));
        Pose2d leftwhitespikemark = new Pose2d(28, -51, Math.toRadians(270));


        drive.setPoseEstimate(startPos);

        //============================
        // TRAJECTORIES
        //============================

        //StageRedLeft
        TrajectorySequence StageRedLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .lineToLinearHeading(StageRedLeftDropoff)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.25)
                .lineToLinearHeading(StageRedLeft)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1point5();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .forward(6)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(.25)
                .lineToLinearHeading(RedWallPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .waitSeconds(0.5)
                .build();



        //StageRedCenter
        TrajectorySequence StageRedCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                //set the gripper to close to pixels and move the lift and angler to deploy right
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerDeploy())
                //go to backstage center position and open the right gripper
                .splineToLinearHeading(StageRedCenterDropoff, Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                //go back and set slide level to 1.5
                .back(15)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1point5();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.2)
                //go to spike mark center and release the pixels
                .splineToLinearHeading(StageRedCenter, Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                //go to the end of the mat to prepare for the spline
                .lineToLinearHeading(StageSplineMidPoint1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1point5();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                //go past the truss
                .splineToLinearHeading(RallyPoint_whitespikemark1, Math.toRadians(270))
                //go to left white spike mark
                .lineToLinearHeading(leftwhitespikemark)
                .waitSeconds(1.5)
                //come back to truss
                .splineToLinearHeading(RallyPoint_whitespikemark2, Math.toRadians(270))
                .lineToLinearHeading(StageSplineMidPoint2)
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerDeploy())
                .splineToLinearHeading(StageRedCenterDropoff2, Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperOpen())
                .waitSeconds(0.25)
                .lineToLinearHeading(RedWallPark)
                .build();

        //StageRedRight
        TrajectorySequence StageRedRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .lineToLinearHeading(StageRedRightDropoff)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.25)
                .lineToLinearHeading(StageRedRight)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerLoad();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .strafeRight(18)
                .lineToLinearHeading(RedWallPark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(1.0)
                .build();



        detector.toggleTelemetry();
        telemetry.clearAll();

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
