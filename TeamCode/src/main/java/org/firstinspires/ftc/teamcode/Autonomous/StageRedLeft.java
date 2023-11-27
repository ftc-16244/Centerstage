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

public class StageRedLeft extends LinearOpMode {
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
        Pose2d RedWallPark = new Pose2d(60,55,Math.toRadians(270));
        Pose2d RedMidPark = new Pose2d(8,50,Math.toRadians(270));

        Pose2d StageRedLeft = new Pose2d(31,19, Math.toRadians(270));
        Pose2d StageRedLeftDropoff = new Pose2d(26,54, Math.toRadians(90));


        drive.setPoseEstimate(startPos);

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

        drive.followTrajectorySequence(StageRedLeftTraj1);
    }
}
