package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Teleop.Centerstage_Teleop2;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.subsytems.PixelDropper;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class StageRedCenter extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    Lift lift = new Lift(this);
    PixelDropper pixelDropper = new PixelDropper(this);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);

        // Initialize the sub systems. Note the init method is inside the subsystem class
        pixelDropper.init(hardwareMap);
        pixelDropper.dropperClosed();

        lift.init(hardwareMap);
        lift.gripperClosed();
        //lift.setAnglerLoad();


        // Move servos to start postion. Grippers open and track wheels up (for teleop)

        // Telemetry

        telemetry.addData("Lift State", null);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pixelDropper.dropperClosed();
        lift.slideMechanicalReset();
        lift.setSlideLevel1();
        lift.setanglerCarry();

        waitForStart();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start of Roadrunner stuff
        Pose2d startPos = new Pose2d(62.5, 12, Math.toRadians(90));

        //Pose2d StageRedLeft1 = new Pose2d(31,18, Math.toRadians(0));
        //Pose2d StageRedLeft2 = new Pose2d(31,-6,Math.toRadians(0));
        Pose2d StageRedCenter = new Pose2d(19,12,Math.toRadians(90));
        //Pose2d StageRedRight = new Pose2d(29,25,Math.toRadians(90));


        Pose2d StageRedCenterDropoff = new Pose2d(39.5, 55, Math.toRadians(90));
        //Pose2d StageRedLeftDropoff = new Pose2d(23,55, Math.toRadians(90));
        //Pose2d StageRedRightDropoff = new Pose2d(46.5, 55, Math.toRadians(90));

        Pose2d RedPark = new Pose2d(7,50,Math.toRadians(90));

        drive.setPoseEstimate(startPos);

        TrajectorySequence StageRedCenterTraj1 = drive.trajectorySequenceBuilder(startPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel2();})
                .lineToLinearHeading(StageRedCenter)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperOpen();})
                .waitSeconds(1)
                .strafeLeft(16)
                .lineToLinearHeading(StageRedCenterDropoff)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.gripperOpen();})
                .waitSeconds(1)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{lift.setSlideLevel1();})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{pixelDropper.dropperClosed();})
                .lineToLinearHeading(RedPark)
                .build();

        drive.followTrajectorySequence(StageRedCenterTraj1);
    }
}
