package org.firstinspires.ftc.teamcode.Autonomous;

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
import org.firstinspires.ftc.teamcode.subsytems.PixelDropper;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class AudBlueWall_Meet2 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(this);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("INFO", "Initializing pipeline");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        WebcamPipeline detector = new WebcamPipeline(telemetry, StartPosition.BLUE_AUD);
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });


        lift.init(hardwareMap);
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

        Pose2d startPos = new Pose2d(-62.5, -36, Math.toRadians(90));
        Pose2d BluePark = new Pose2d(-13,45,Math.toRadians(90));
        Pose2d BlueRallyPoint1 = new Pose2d(-7.5,-41,Math.toRadians(90));
        Pose2d BlueRallyPoint2 = new Pose2d(-7.5,0,Math.toRadians(90));

        Pose2d AudBlueLeftPre = new Pose2d(-29.5,-41,Math.toRadians(90));//spike mark
        Pose2d AudBlueLeft = new Pose2d(-29.5,-32.5,Math.toRadians(90));//spike mark
        Pose2d AudBlueLeftDropoff = new Pose2d(-42, 53, Math.toRadians(90));//backstage

        Pose2d AudBlueCenter = new Pose2d(-21, -43, Math.toRadians(90));
        Pose2d AudBlueCenterDropoff = new Pose2d(-36, 52, Math.toRadians(90));

        Pose2d AudBlueRight = new Pose2d(-20,-57, Math.toRadians(90));
        Pose2d AudBlueRightDropoff = new Pose2d(-34,52, Math.toRadians(90));

        drive.setPoseEstimate(startPos);

        TrajectorySequence StageBlueLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5();})
                .lineToLinearHeading(AudBlueLeftPre)
                .lineToLinearHeading(AudBlueLeft)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperLeftOpen();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerCarry();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(BlueRallyPoint1)
                .lineToLinearHeading(BlueRallyPoint2)
                .lineToLinearHeading(AudBlueLeftDropoff)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel2())
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.125)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(BluePark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.125)
                .build();

        TrajectorySequence StageBlueCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5();})
                .lineToLinearHeading(AudBlueCenter)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperLeftOpen();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerCarry();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(BlueRallyPoint1)
                .lineToLinearHeading(BlueRallyPoint2)
                .lineToLinearHeading(AudBlueCenterDropoff)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel2())
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(BluePark)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.125)
                .build();

        TrajectorySequence StageBlueRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5();})
                .lineToLinearHeading(AudBlueRight)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperLeftOpen();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerCarry();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .strafeRight(6)
                .lineToLinearHeading(BlueRallyPoint1)
                .lineToLinearHeading(BlueRallyPoint2)
                .lineToLinearHeading(AudBlueRightDropoff)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel2())
                .forward(2.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.125)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperWideOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(BluePark)
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

        telemetry.addData("Running path", " BLUE_AUD_" + location);
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
