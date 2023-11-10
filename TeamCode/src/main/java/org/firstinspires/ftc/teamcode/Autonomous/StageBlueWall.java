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
public class StageBlueWall extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(this);
        PixelDropper pixelDropper = new PixelDropper(this);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("INFO", "Initializing pipeline");
        telemetry.update();
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

        pixelDropper.init(hardwareMap);
        pixelDropper.dropperClosed();

        lift.init(hardwareMap);
        lift.setAnglerLoad();
        sleep(250);
        lift.gripperClosed();
        pixelDropper.dropperClosed();
        lift.slideMechanicalReset();
        lift.setAnglerCarry();
        sleep(250); // no sleepy no workie. Need this to let the anger servo have time to move

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

        Pose2d startPos = new Pose2d(-62.5, 12, Math.toRadians(90));
        Pose2d BluePark = new Pose2d(-55,50,Math.toRadians(90));

        // Center Prop
        // pixel drop point
        Pose2d StageBlueCenter = new Pose2d(-17.5,12,Math.toRadians(90));
        //backstage drop
        Pose2d StageBlueCenterDropoff = new Pose2d(-38, 50, Math.toRadians(90));

        // Left Prop Poses - this one has an extra motion
        // Strafe and rotate towards drive team.
        Pose2d StageBlueRight1 = new Pose2d(-33.5,18, Math.toRadians(180));
        // Strafe under the truss partially to drop the pixel
        Pose2d StageBlueRight2 = new Pose2d(-30.5,-5, Math.toRadians(180));
        // Position on the backstage board to drop yellow pixel
        Pose2d StageBlueRightDropoff = new Pose2d(-24.5,53, Math.toRadians(90));

        // Right Prop Poses

        Pose2d StageBlueLeft = new Pose2d(-24,25,Math.toRadians(90));
        Pose2d StageBlueLeftDropoff = new Pose2d(-46, 51, Math.toRadians(90));


        drive.setPoseEstimate(startPos);

        TrajectorySequence StageBlueRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .lineToLinearHeading(StageBlueRightDropoff)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperOpen();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setAnglerCarry();})
                .lineToLinearHeading(StageBlueRight1)
                .lineToLinearHeading(StageBlueRight2)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperOpen();})
                .waitSeconds(0.25)
                .strafeLeft(5)
                .build();

        TrajectorySequence StageBlueCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .lineToLinearHeading(StageBlueCenter)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperOpen();})
                .waitSeconds(0.25)
                .strafeLeft(22)
                .lineToLinearHeading(StageBlueCenterDropoff)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperOpen();})
                .waitSeconds(0.5)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperClosed();})
                .lineToLinearHeading(BluePark)
                .build();

        TrajectorySequence StageBlueLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .lineToLinearHeading(StageBlueLeft)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{pixelDropper.dropperOpen();})
                .waitSeconds(0.25)
                .strafeLeft(19)
                .lineToLinearHeading(StageBlueLeftDropoff)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperOpen();})
                .waitSeconds(0.25)
                .UNSTABLE_addDisplacementMarkerOffset(0.0, ()->{pixelDropper.dropperClosed();})
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .lineToLinearHeading(BluePark)
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
                drive.followTrajectorySequence(StageBlueLeftTraj1);
                break;
            case CENTER:
                drive.followTrajectorySequence(StageBlueCenterTraj1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(StageBlueRightTraj1);
                lift.gripperClosed();
                lift.setSlideLevel1();
                lift.setAnglerLoad();
                break;
            default:
                throw new IllegalArgumentException("The code is most certainly severely screwed up.");
        }
    }
}
