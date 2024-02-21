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
import org.firstinspires.ftc.teamcode.subsytems.Drone;
import org.firstinspires.ftc.teamcode.subsytems.Felipe2;
import org.firstinspires.ftc.teamcode.subsytems.Juan;

@Config
@TeleOp(group = "Teleop")

public class State_Teleop extends LinearOpMode {
    Juan juan = new Juan(this);
    Felipe2 felipe = new Felipe2(this); // replaces climberDone with climber only subsystem
    Drone drone = new Drone(this); // replaces climberDone with drone only subsystem

    private ElapsedTime teleopTimer = new ElapsedTime();
    private final float TELEOP_TIME_OUT = 140; // WARNING: LOWER FOR OUTREACH

    FtcDashboard dashboard;
    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern preInit = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
    RevBlinkinLedDriver.BlinkinPattern main = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
    RevBlinkinLedDriver.BlinkinPattern endgame = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
    RevBlinkinLedDriver.BlinkinPattern climbAlert = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;

    double speedFactor = 1.0;
    @Override
    public void runOpMode() throws InterruptedException {
        // set up Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the subsystems. Note the init method is inside the subsystem class
        felipe.init(hardwareMap);
        felipe.gripperWideOpen();
        felipe.setAnglerLoad();

        drone.init(hardwareMap);
        drone.setDroneGrounded(); // power servo to make sure rubber band stays tight.

        // no need to power the hook servo until it is time to climb

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        blinkin.setPattern(preInit);

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

        waitForStart();
        teleopTimer.reset();

        Thread liftInit = new Thread(() -> {
            felipe.gripperOpen(); // put gripper in open position. Not super wide open
            felipe.setAnglerDeploy();
            felipe.startSlideMechanicalReset();
            sleep(4000);
        });
        liftInit.start();

        Thread climberInit = new Thread(() -> {
            juan.init(hardwareMap);
            sleep(5000);
        });
        climberInit.start();

        blinkin.setPattern(main);

        while (!isStopRequested() && teleopTimer.time() < TELEOP_TIME_OUT) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            scaleStick(-gamepad1.left_stick_y) * speedFactor,
                            scaleStick(-gamepad1.left_stick_x) * speedFactor,
                            scaleStick(-gamepad1.right_stick_x) * speedFactor
                    )
            );
            if (teleopTimer.time() > 90 && !(teleopTimer.time() > 110)) { // endgame
                blinkin.setPattern(endgame);
            }
            if (teleopTimer.time() > 110 && !(teleopTimer.time() > 120)) { // climb alert (10 seconds to climb)
                blinkin.setPattern(climbAlert);
            }
            if (teleopTimer.time() > 120) {
                blinkin.close();
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
                setManarMode(0.85, true);
            }

            if (gamepad1.right_trigger > 0.25) {
                gp1righttrigger();
            }

            if (gamepad1.right_bumper) {
                felipe.gripperLeftOpen();
                sleep(100);
            }

            if (gamepad1.left_bumper) {
                felipe.gripperRightOpen();
                sleep(100);
            }
            if (gamepad1.left_stick_button) {
                speedFactor = 0.5;
            }
            if (gamepad1.right_stick_button) {
                speedFactor = 0.25;
            }
            if (gamepad1.a) {
                speedFactor = 0.85;
            }
//// GAMEPAD #2/////////////////////////

            if (gamepad2.a) {
                gp2a();
            }
            if (gamepad2.y) {
                gp2y();
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
                setManarMode(1.0, true);
                sleep(100);
            }
            if (gamepad2.dpad_right) {
                gp2dpright();
                setManarMode(0.5, false);
                sleep(100);
            }
            if (gamepad2.dpad_up) {
                gp2dpup();
                setManarMode(0.5, false);
                sleep(100);
            }
            if (gamepad2.dpad_left) {
                gp2dpleft();
                setManarMode(0.5, false);
                sleep(100);
            }
            if (gamepad2.left_bumper) {

                gp2leftbumper();
                setManarMode(1.0, true);
                sleep(100);
            }
            if (gamepad2.right_bumper) {
                //felipe.setTurnerDeploy();
                gp2rightbumper();
                setManarMode(0.5, false);
                sleep(100);
            }
            if (gamepad2.left_trigger > 0.25) {
                gp2lefttrigger();
                setManarMode(1.0, true);
                sleep(100);
            }
            if (gamepad2.right_trigger > 0.25) {
                setManarMode(0.5, false);
                felipe.setAnglerDeploy();
                //gp2righttrigger();
                sleep(100);
            }
        }
    }
    private void gp1lefttrigger() {
        Thread gp1lefttrigger = new Thread(() -> {
            felipe.gripperOpen();
            sleep(200);
        });
        gp1lefttrigger.start();
    }
    private void gp1righttrigger() {
        Thread gp1righttrigger = new Thread(() -> {
            felipe.gripperClosed();
            sleep(500);
            felipe.setAnglerDeploy();
            sleep(200);
        });
        gp1righttrigger.start();
    }
    private void gp2a() {
        Thread gp2a = new Thread(() -> {
            juan.climb();
            sleep(500);
        });
        gp2a.start();
    }
    private void gp2b() {
        Thread gp2b = new Thread(() -> {
            juan.reset();
            sleep(500);
        });
        gp2b.start();
    }
    private void gp2y() {
        Thread gp2y = new Thread(() -> {
            juan.prepForClimb();
            sleep(500);
        });
        gp2y.start();
    }
    private void gp2dpup() {
        Thread gp2dpup = new Thread(() -> {
            if(felipe.turnerDown) {
                felipe.setSlideReach_2();
            } else {
                felipe.setSlideRow_4();
            }
            sleep(200);
        });
        gp2dpup.start();
    }
    private void gp2dpdown() {
        Thread gp2dpdown = new Thread(() -> {
            felipe.setSlideLevel_0();
            sleep(200);
        });
        gp2dpdown.start();
    }
    private void gp2dpleft() {
        Thread gp2dpleft = new Thread(() -> {
            if(felipe.turnerDown) {
                felipe.setSlideReach_3();
            } else {
                felipe.setSlideRow_6();
            }
            sleep(200);
        });
        gp2dpleft.start();
    }
    private void gp2dpright() {
        Thread gp2dpright = new Thread(() -> {
            if(felipe.turnerDown) {
                felipe.setSlideReach_1();
            } else {
                felipe.setSlideRow_2();
            }
            sleep(200);
        });
        gp2dpright.start();
    }
    private void gp2lefttrigger() {
        Thread gp2lefttrigger = new Thread(() -> {
            //felipe.setAnglerDeploy();
            felipe.setAnglerLoad();
            felipe.gripperOpen();
            sleep(100);
        });
        gp2lefttrigger.start();
    }
    private void gp2righttrigger() {
        Thread gp2righttrigger = new Thread(() -> {
            felipe.setAnglerDeploy();

            sleep(100);
        });
        gp2righttrigger.start();
    }
    private void gp2leftbumper() {
        Thread gp2leftbumper = new Thread(() -> {
            felipe.setTurnerLoad();
            sleep(1000);
        });
        gp2leftbumper.start();
    }
    private void gp2rightbumper() {
        Thread gp2rightbumper = new Thread(() -> {
            felipe.setTurnerDeploy();
            sleep(1000);
        });
        gp2rightbumper.start();
    }
    private double scaleStick(double input) {
        if(input < 0.75 && input > -0.75) return input / 1.5;
        else return input;
    }
    private void setManarMode(double newSpeedFactor, boolean overwrite) {
        if (newSpeedFactor < speedFactor || overwrite) {
            speedFactor = newSpeedFactor;
        }
    }
}