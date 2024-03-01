package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Drone;
import org.firstinspires.ftc.teamcode.subsytems.Felipe2;
import org.firstinspires.ftc.teamcode.subsytems.Juan;

@Config
@Disabled
@TeleOp(group = "Test")

public class TurnerTest extends LinearOpMode {
    Felipe2 felipe = new Felipe2(this); // replaces climberDone with climber only subsystem
    Drone drone = new Drone(this); // replaces climberDone with drone only subsystem

    FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the subsystems. Note the init method is inside the subsystem class
        felipe.init(hardwareMap);
        felipe.gripperWideOpen();
        felipe.setAnglerLoad();

        drone.init(hardwareMap);
        drone.setDroneGrounded(); // power servo to make sure rubber band stays tight.

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

        waitForStart();

        Thread liftInit = new Thread(() -> {
            felipe.gripperOpen(); // put gripper in open position. Not super wide open
            felipe.setAnglerDeploy();
            felipe.startSlideMechanicalReset();
            sleep(4000);
        });
        liftInit.start();

        while (!isStopRequested()) {
            if(gamepad2.x) {
                felipe.setTurnerDrone();
                sleep(100);
                drone.setDroneFly();
            }
            if(gamepad2.left_bumper) {
                felipe.setTurnerLoad();
            }
            if(gamepad2.right_bumper) {
                felipe.setTurnerDeploy();
            }
            if(gamepad1.a) {
                felipe.setTurnerDeployAUTOWHITESTACK();
            }
        }
    }
}