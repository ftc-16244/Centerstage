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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class StageBlue extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        WebcamPipeline detector = new WebcamPipeline(telemetry, StartPosition.BLUE_STAGE);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}

        });

        waitForStart();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start of Roadrunner stuff
        Pose2d startPos = new Pose2d(60, 24, Math.toRadians(90));

        Pose2d StageBlueLeft= new Pose2d(-30,12,Math.toRadians(90));
        Pose2d StageBlueCenter = new Pose2d(-21.5,24,Math.toRadians(90));
        Pose2d StageBlueRight = new Pose2d(-30,40,Math.toRadians(90));


        Pose2d StageBlueCenterDropoff = new Pose2d(-34, 60, Math.toRadians(90));
        Pose2d StageBlueLeftDropoff = new Pose2d(-29, 60, Math.toRadians(90));
        Pose2d StageBlueRightDropoff = new Pose2d(-39, 60, Math.toRadians(90));

        Pose2d BluePark = new Pose2d(-12,60,Math.toRadians(90));

        drive.setPoseEstimate(startPos);


        //StageRedLeft
        TrajectorySequence StageBlueLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                .strafeRight(27)
                .waitSeconds(1)
                .back(4)
                .waitSeconds(1)
                .lineToLinearHeading(StageBlueLeftDropoff)
                .waitSeconds(1)
                .lineToLinearHeading(BluePark)
                .build();

        //StageRedCenter
        TrajectorySequence StageBlueCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(StageBlueCenter)
                .waitSeconds(1)
                .lineToLinearHeading(StageBlueCenterDropoff)
                .waitSeconds(1)
                .lineToLinearHeading(BluePark)
                .build();

        //StageRedRight
        TrajectorySequence StageBlueRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(StageBlueRight)
                .waitSeconds(1)
                .lineToLinearHeading(StageBlueRightDropoff)
                .waitSeconds(1)
                .lineToLinearHeading(BluePark)
                .build();

        switch(detector.getPropLocation()){
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
                //if camera cannot detect, runs StageRedCenter trajectory
                drive.followTrajectorySequence(StageBlueCenterTraj1);
                break;
        }

    }
}
