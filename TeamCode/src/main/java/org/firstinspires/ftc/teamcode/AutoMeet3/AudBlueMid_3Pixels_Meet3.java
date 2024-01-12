package org.firstinspires.ftc.teamcode.AutoMeet3;

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
public class AudBlueMid_3Pixels_Meet3 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() throws InterruptedException {

        Lift lift = new Lift(this);

        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        Pose2d startPos = new Pose2d(-62.5, -36, Math.toRadians(90));
        Pose2d RedPrePark = new Pose2d(13,45,Math.toRadians(90));
        Pose2d BluePark = new Pose2d(-18,51,Math.toRadians(90));
        Pose2d BlueRallyPoint1_LEFT = new Pose2d(-10.5,-55,Math.toRadians(90));//left and a bit forward from the left pixel drop
        Pose2d BlueRallyPoint1_CENTER = new Pose2d(-9.75,-40,Math.toRadians(90));//left and a bit forward from the center pixel drop
        Pose2d BlueRallyPoint1_RIGHT = new Pose2d(-10.5,-40,Math.toRadians(90));//left and a bit forward from the center pixel drop
        Pose2d BlueRallyPoint2 = new Pose2d(-10.5,-0,Math.toRadians(90));// near md field on red side

        //Center Prop
        Pose2d AudBlueCenter = new Pose2d(-21.5, -45, Math.toRadians(90));
        Pose2d AudBlueCenterPush = new Pose2d(-22.5, -45.5, Math.toRadians(90));
        //backstage drop
        Pose2d AudBlueCenterDropoff = new Pose2d(-38.5, 53.5, Math.toRadians(90)); // was 39

        // right Prop Poses
        Pose2d AudBlueRight = new Pose2d(-32,-30.5, Math.toRadians(270)); // y was -32.5
        Pose2d AudBlueRightPush = new Pose2d(-25,-32.5, Math.toRadians(270)); // y was 57.5
        Pose2d AudBlueRightDropoff = new Pose2d(-33,53, Math.toRadians(90)); // x was -29

        // left Prop Poses
        Pose2d AudBlueLeftDropoff = new Pose2d(-41.4, 53, Math.toRadians(90));//backstage
        Pose2d AudBlueLeftPre = new Pose2d(-32.5,-41, Math.toRadians(90));//pushing the stage prop
        Pose2d AudBlueLeftPush = new Pose2d(-32.5,-25,Math.toRadians(90));//spike mark
        Pose2d AudBlueLeft = new Pose2d(-32.5,-35,Math.toRadians(90));//spike mark

        //white spike mark
        Pose2d leftwhitespikemark_BLUE_LEFT = new Pose2d(-15.5, -45.5, Math.toRadians(270));
        Pose2d leftwhitespikemark_BLUE_CENTER = new Pose2d(-14, -47.5, Math.toRadians(270));
        Pose2d leftwhitespikemark_BLUE_RIGHT = new Pose2d(-16.25, -48.5, Math.toRadians(270));


        drive.setPoseEstimate(startPos);

        //============================
        // TRAJECTORIES
        //============================

        //StageRedLeft
        TrajectorySequence StageRedLeftTraj1 = drive.trajectorySequenceBuilder(startPos)
                //set the gripper to close to pixels and move the lift and angler to deploy right
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5_white();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerDeploy())
                //go to push the pixel away and then go to left=  position and open the left gripper
                .lineToLinearHeading(AudBlueLeftPre)
                .lineToLinearHeading(AudBlueLeftPush)
                .lineToLinearHeading(AudBlueLeft)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.75)
                //head back to avoid hitting pixels and then spline to white pixel spike mark
                .lineToLinearHeading(leftwhitespikemark_BLUE_LEFT)
                //pick up the white pixel and then go back
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperClosed())
                .waitSeconds(0.25)
                .back(10)
                //set slide to 1 and put angler in carry position
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerCarry())
                //spline to backboard
                .lineToLinearHeading(BlueRallyPoint1_LEFT)
                .splineToLinearHeading(BlueRallyPoint2, Math.toRadians(90))
                .splineToLinearHeading(AudBlueLeftDropoff,Math.toRadians(90))
                //set slide to 2 and open right gripper
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1point5_back())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                //go back 6 in and then set slide level to 1
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                //head over to park
                .lineToLinearHeading(BluePark)
                .build();

        //StageRedCenter
        TrajectorySequence StageRedCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                //set the gripper to close to pixels and move the lift and angler to deploy right
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5_white();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerDeploy())
                //go to push the pixel away and then go to center position and open the left gripper
                .lineToLinearHeading(AudBlueCenter)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                //head back to avoid hitting pixels and then spline to white pixel spike mark
                .back(3)
                //.splineToLinearHeading(leftwhitespikemark_BLUE, Math.toRadians(270))
                .lineToLinearHeading(leftwhitespikemark_BLUE_CENTER)
                //pick up the white pixel and then go back
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperClosed())
                .waitSeconds(0.25)
                .back(6.5)
                //set slide to 1 and put angler in carry position
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerCarry())
                //spline to backboard
                .splineToLinearHeading(BlueRallyPoint1_CENTER, Math.toRadians(90))
                .splineToLinearHeading(BlueRallyPoint2, Math.toRadians(90))
                .splineToLinearHeading(AudBlueCenterDropoff,Math.toRadians(90))
                //set slide to 2 and open right gripper
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1point5_back())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                //go back 6 in and then set slide level to 1
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                //head over to park
                .lineToLinearHeading(BluePark)
                .build();


        //StageRedRight
        TrajectorySequence StageRedRightTraj1 = drive.trajectorySequenceBuilder(startPos)
                //set the gripper to close to pixels and move the lift and angler to deploy right
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperClosed();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{lift.setSlideLevel1point5_white();})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->lift.setAnglerLoad())
                //go to push the pixel away and then go to right position and open the left gripper
                .lineToLinearHeading(AudBlueRightPush)
                .lineToLinearHeading(AudBlueRight)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                //head back to avoid hitting pixels and then spline to white pixel spike mark
                .strafeLeft(13)
                .lineToLinearHeading(leftwhitespikemark_BLUE_RIGHT)
                //pick up the white pixel and then go back
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperClosed())
                .waitSeconds(0.25)
                .back(10)
                //set slide to 1 and put angler in carry position
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerCarry())
                //spline to backboard
                .splineToLinearHeading(BlueRallyPoint1_RIGHT, Math.toRadians(90))
                .splineToLinearHeading(BlueRallyPoint2, Math.toRadians(90))
                .splineToLinearHeading(AudBlueRightDropoff,Math.toRadians(90))
                //set slide to 2 and open right gripper
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1point5_back())
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                //go back 6 in and then set slide level to 1
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setSlideLevel1())
                //head over to park
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

        telemetry.addData("Running path", " BLUE_AUD" + location);
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
