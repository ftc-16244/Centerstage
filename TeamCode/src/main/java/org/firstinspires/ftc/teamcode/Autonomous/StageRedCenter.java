package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class StageRedCenter extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        waitForStart();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start of Roadrunner stuff
        Pose2d startPos = new Pose2d(60, 24, Math.toRadians(90));
        Pose2d StageRedLeft= new Pose2d(30,0,Math.toRadians(0));
        Pose2d StageRedCenter = new Pose2d(21.5,24,Math.toRadians(90));
        Pose2d StageRedRight = new Pose2d(30,24,Math.toRadians(0));
        Pose2d StageRedCenterDropoff = new Pose2d(34, 60, Math.toRadians(90));
        Pose2d RedPark = new Pose2d(12,60,Math.toRadians(90));

        drive.setPoseEstimate(startPos);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPos)
                .lineToLinearHeading(StageRedCenter)
                .waitSeconds(1)
                .lineToLinearHeading(StageRedCenterDropoff)
                .waitSeconds(1)
                .lineToLinearHeading(RedPark)
                .build();

        drive.followTrajectorySequence(traj1);
    }
}
