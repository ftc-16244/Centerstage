package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Climber;
import org.firstinspires.ftc.teamcode.subsytems.Drone;
import org.firstinspires.ftc.teamcode.subsytems.Felipe2;
import org.firstinspires.ftc.teamcode.subsytems.Juan;
import org.firstinspires.ftc.teamcode.subsytems.Lift;

@Config
@TeleOp(group = "Teleop")

public class State_Teleop extends LinearOpMode {
    Juan juan = new Juan(this);
    Felipe2 felipe = new Felipe2(this); // replaces climberDone with climber only subsystem
    Drone drone = new Drone(this); // replaces climberDone with drone only subsystem

    private ElapsedTime teleopTimer = new ElapsedTime();
    private double TELEOP_TIME_OUT = 140; // WARNING: LOWER FOR OUTREACH

    FtcDashboard dashboard;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern preInit;
    RevBlinkinLedDriver.BlinkinPattern main;
    RevBlinkinLedDriver.BlinkinPattern endgame;
    RevBlinkinLedDriver.BlinkinPattern climbAlert;

    @Override
    public void runOpMode() throws InterruptedException {
        // set up local variables
        double  speedFactor = 0.75;

        // set up Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the subsystems. Note the init method is inside the subsystem class
        felipe.init(hardwareMap);
        felipe.gripperWideOpen();
        felipe.setAnglerLoad();

        Thread climberInit = new Thread(() -> {
            juan.init(hardwareMap);
            juan.reset();
        });
        climberInit.start();
        drone.init(hardwareMap);
        drone.setDroneGrounded(); // power servo to make sure rubber band stays tight.

        // no need to power the hook servo until it is time to climb//

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        felipe.slideMechanicalReset();
        felipe.setSlideLevel_0();

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        preInit = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        main = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
        endgame = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
        climbAlert = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;

        blinkinLedDriver.setPattern(preInit);

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

        waitForStart();
        teleopTimer.reset();

        blinkinLedDriver.setPattern(main);

        felipe.gripperOpen(); // put gripper in open position. Not super wide open

        while (!isStopRequested() && teleopTimer.time() < TELEOP_TIME_OUT) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            scaleStick(-gamepad1.left_stick_y) * speedFactor,
                            scaleStick(-gamepad1.left_stick_x) * speedFactor,
                            scaleStick(-gamepad1.right_stick_x) * speedFactor
                    )
            );
            if (teleopTimer.time() > 90 && !(teleopTimer.time() > 110)) { // endgame
                blinkinLedDriver.setPattern(endgame);
            }
            if (teleopTimer.time() > 110 && !(teleopTimer.time() > 120)) { // climb alert (10 seconds to climb)
                blinkinLedDriver.setPattern(climbAlert);
            }
            if (teleopTimer.time() > 120) {
                blinkinLedDriver.close();
            }

            if (gamepad1.dpad_right) {

            }

            if (gamepad1.dpad_up) {

            }

            if (gamepad1.dpad_down) {

            }

            if (gamepad1.dpad_left) {


            }

            if (gamepad1.back) {

            }
            if (gamepad1.left_trigger > 0.25) {
                gp1lefttrigger();
                speedFactor = 1.0;
            }

            if (gamepad1.right_trigger > 0.25) {
                gp1righttrigger();
                sleep(100);
            }

            if (gamepad1.right_bumper) {
                felipe.gripperRightOpen();
                sleep(100);
            }

            if (gamepad1.left_bumper) {
                felipe.gripperLeftOpen();
                sleep(100);
            }
            if (gamepad1.left_stick_button) {
                speedFactor = 0.5;
            }
            if (gamepad1.right_stick_button) {
                speedFactor = 0.25;
            }
            if (gamepad1.a) {
                speedFactor = 0.75;
            }
            if (gamepad1.y) {
                speedFactor = 1.0;
            }
//// GAMEPAD #2/////////////////////////

            if (gamepad2.a) {
                gp2a();
                sleep(500);
            }
            if (gamepad2.y) {
                gp2y();
                sleep(500);
            }
            if (gamepad2.b) {
                gp2b();
            }
            if (gamepad2.back) {
                felipe.slideMechanicalReset();
            }
            if (gamepad2.x) {
                drone.setDroneFly(); // move servo to let drone go
                sleep(50); // pause to make sure servo moves
            }
            if (gamepad2.dpad_down) {
                gp2dpdown();
                speedFactor = 0.75;
                sleep(200);
            }
            if (gamepad2.dpad_right) {
                gp2dpright();
                speedFactor = 0.5;
                sleep(200);
            }
            if (gamepad2.dpad_up) {
                gp2dpup();
                speedFactor = 0.5;
                sleep(200);
            }
            if (gamepad2.dpad_left) {
                gp2dpleft();
                speedFactor = 0.5;
                sleep(200);
            }
            if (gamepad2.left_trigger > 0.25) {
                gp2lefttrigger();
                speedFactor = 0.75;
                sleep(100);
            }
            if (gamepad2.right_trigger > 0.25) {
                speedFactor = 0.5;
                gp2righttrigger();
                sleep(100);
            }
        }
    }
    private void gp1lefttrigger() {
        Thread gp1lefttrigger = new Thread(() -> {
            felipe.gripperClosed();
            sleep(500);
            felipe.setAnglerDeploy();
        });
        gp1lefttrigger.start();
    }
    private void gp1righttrigger() {
        Thread gp1righttrigger = new Thread(() ->
                felipe.gripperOpen()
        );
        gp1righttrigger.start();
    }
    private void gp2dpup() {
        Thread gp2dpup = new Thread(() ->
                felipe.setSlideRow_2()
        );
        gp2dpup.start();
    }
    private void gp2a() {
        Thread gp2a = new Thread(() ->
            juan.climb()
        );
        gp2a.start();
    }
    private void gp2b() {
        Thread gp2b = new Thread(() ->
                juan.reset()
        );
        gp2b.start();
    }
    private void gp2y() {
        Thread gp2y = new Thread(() ->
            juan.prepForClimb()
        );
        gp2y.start();
    }
    private void gp2dpdown() {
        Thread gp2dpdown = new Thread(() ->
                felipe.setSlideLevel_0()
        );
        gp2dpdown.start();
    }
    private void gp2dpleft() {
        Thread gp2dpleft = new Thread(() ->
                felipe.setSlideRow_6()
        );
        gp2dpleft.start();
    }
    private void gp2dpright() {
        Thread gp2dpright = new Thread(() -> felipe.setSlideRow_4());
        gp2dpright.start();
    }
    private void gp2lefttrigger() {
        Thread gp2lefttrigger = new Thread(() -> felipe.setAnglerDeploy());
        gp2lefttrigger.start();
    }
    private void gp2righttrigger() {
        Thread gp2righttrigger = new Thread(() -> {
            felipe.setAnglerLoad();
           felipe.gripperOpen();
        });
        gp2righttrigger.start();
    }
    private double scaleStick(double input) {
        if(input < 0.75 && input > -0.75) return input / 1.5;
        else return input;
    }
}
