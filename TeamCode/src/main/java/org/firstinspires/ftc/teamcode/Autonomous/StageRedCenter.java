package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.subsytems.PixelDropper;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class StageRedCenter extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    Lift lift = new Lift(this);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        // Initialize the sub systems. Note the init method is inside the subsystem class

        lift.init(hardwareMap);
        lift.gripperClosed();
        //lift.setAnglerLoad();


        // Move servos to start postion. Grippers open and track wheels up (for teleop)

        // Telemetry

        telemetry.addData("Lift State", null);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift.slideMechanicalReset();
        lift.setSlideLevel1();
        lift.setAnglerLoad();

        waitForStart();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start of Roadrunner stuff
        Pose2d startPos = new Pose2d(62.5, 12, Math.toRadians(90));
        Pose2d RedWallPark = new Pose2d(63,50,Math.toRadians(90));
        Pose2d RedMidPark = new Pose2d(8,50,Math.toRadians(90));

        //Center Prop
        Pose2d StageRedCenter = new Pose2d(30, 19, Math.toRadians(180));
        //backstage drop
        Pose2d StageRedCenterDropoff = new Pose2d(35, 54, Math.toRadians(90));

        drive.setPoseEstimate(startPos);

        TrajectorySequence StageRedCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerDeploy())
                .lineToLinearHeading(StageRedCenterDropoff)
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperRightOpen())
                .waitSeconds(0.25)
                .lineToLinearHeading(StageRedCenter)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.setAnglerLoad())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->lift.gripperLeftOpen())
                .waitSeconds(0.5)
                .back(4)
                .lineToLinearHeading(RedWallPark)
                .build();

        drive.followTrajectorySequence(StageRedCenterTraj1);
    }
}
