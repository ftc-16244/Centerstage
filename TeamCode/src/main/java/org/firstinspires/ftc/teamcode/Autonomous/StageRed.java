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
public class StageRed extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

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

        waitForStart();

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

        // Start of Roadrunner stuff
        Pose2d startPos = new Pose2d(60, 24, Math.toRadians(90));

        Pose2d StageRedLeft = new Pose2d(30,12,Math.toRadians(90));
        Pose2d StageRedCenter = new Pose2d(21.5,24,Math.toRadians(90));
        Pose2d StageRedRight = new Pose2d(30,40,Math.toRadians(90));


        Pose2d StageRedCenterDropoff = new Pose2d(34, 60, Math.toRadians(90));
        Pose2d StageRedLeftDropoff = new Pose2d(29, 60, Math.toRadians(90));
        Pose2d StageRedRightDropoff = new Pose2d(39, 60, Math.toRadians(90));

        Pose2d RedPark = new Pose2d(12,60,Math.toRadians(90));

        drive.setPoseEstimate(startPos);


        //StageRedLeft
        TrajectorySequence StageRedLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                .strafeLeft(27)
                .waitSeconds(1)
                .back(4)
                .waitSeconds(1)
                .lineToLinearHeading(StageRedLeftDropoff)
                .waitSeconds(1)
                .lineToLinearHeading(RedPark)
                .build();

        //StageRedCenter
        TrajectorySequence StageRedCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(StageRedCenter)
                .waitSeconds(1)
                .lineToLinearHeading(StageRedCenterDropoff)
                .waitSeconds(1)
                .lineToLinearHeading(RedPark)
                .build();

        //StageRedRight
        TrajectorySequence StageRedRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(StageRedRight)
                .waitSeconds(1)
                .lineToLinearHeading(StageRedRightDropoff)
                .waitSeconds(1)
                .lineToLinearHeading(RedPark)
                .build();

        detector.toggleTelemetry();
        telemetry.clearAll();

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
