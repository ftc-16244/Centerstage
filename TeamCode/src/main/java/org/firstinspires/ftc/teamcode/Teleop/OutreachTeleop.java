package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.LiftGripper;


@Config
@TeleOp(group = "Teleop")

public class OutreachTeleop extends LinearOpMode {
// test

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime turnerTimer = new ElapsedTime();

    LiftGripper liftGripper = new LiftGripper(this);

    private ElapsedTime teleopTimer = new ElapsedTime();
    private double TELEOP_TIME_OUT = 130;


    FtcDashboard dashboard;


    //ENUMS
    public enum LiftState {
        LIFT_UNKNOWN,
        LIFT_IDLE,
        LIFT_LOW,
        LIFT_MED,
        LIFT_HIGH,
        LIFT_TURNER_FRONT,
        LIFT_TURNER_BACK,
        LIFT_HOLD
    }
    public enum AnglerState {
        Deposit,
        Collect,
        Other
    }


    LiftState liftState = LiftState.LIFT_UNKNOWN;
    AnglerState anglerState;
    boolean heightLow;

    @Override
    public void runOpMode() throws InterruptedException {


        // set up local variables

        double  slidePosition;
        double  speedFactor = 0.5;
        double expo =   3; // has to be 1 or 3

        // set up Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the sub systems. Note the init method is inside the subsystem class


        // Move servos to start postion. Grippers open and track wheels up (for teleop)

        liftState = LiftState.LIFT_IDLE;
        anglerState = AnglerState.Deposit;
        heightLow = false;
        turnerTimer.reset();

        // Telemetry

        telemetry.addData("Lift State", null);
        telemetry.addData("Angler State", anglerState);
        telemetry.addData("Height Low", heightLow);
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

        waitForStart();

        while (!isStopRequested() && teleopTimer.time() < TELEOP_TIME_OUT) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            Math.pow(-gamepad1.left_stick_y, expo) * speedFactor,
                            Math.pow(-gamepad1.left_stick_x,expo) * speedFactor,
                            Math.pow(-gamepad1.right_stick_x,expo) * speedFactor
                    )
            );

            if (teleopTimer.time() > 93){


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
            if (gamepad1.a) {
                if (speedFactor == 1) speedFactor = 0.5;
                else speedFactor = 1;
            }
            if (gamepad1.left_trigger > 0.25) {

            }

            if (gamepad1.right_trigger > 0.25) {

            }
//// GAMEPAD #2/////////////////////////

            if (gamepad2.dpad_up) {

            }
            /*
            switch(liftState) {
                case LIFT_IDLE:
                    //do nothing, waiting for driver input

                    //event and exit condition
                    if (gamepad2.dpad_right) {
                        slideTrainer.setSlideLevel5();
                        liftState = LiftState.LIFT_HIGH;
                    }
                    else if (gamepad2.dpad_up) {
                        slideTrainer.setSlideLevel4();
                        liftState = LiftState.LIFT_MED;
                    }
                    else if (gamepad2.dpad_left) {
                        slideTrainer.setSlideLevel3();
                        liftState = LiftState.LIFT_LOW;
                    }
                    else if (gamepad2.left_trigger > 0.25 || gamepad2.right_trigger > 0.25) {
                        slideTrainer.setSlideLevel1();
                        liftState = LiftState.LIFT_GET_CONE;
                    }
                    break;

                case LIFT_HIGH:

                    //action: check if height is within half inch of target, if not wait until it is
                    if (Math.abs(slideTrainer.getSlidePos() - slideTrainer.SLIDE_LEVEL_5) < 24) {
                        if (turnerState != TurnerState.FORWARD) {
                            turnerTimer.reset();
                            gripper.turnerSetPosition2(); //fwd
                            liftState = LiftState.LIFT_TURNER_FRONT;
                        }
                        else {
                            liftState = LiftState.LIFT_HOLD;
                        }
                    }
                    break;

                case LIFT_MED:
                    //action: check if height is within half inch of target, if not wait until it is
                    if (Math.abs(slideTrainer.getSlidePos() - slideTrainer.SLIDE_LEVEL_4) < 12) {
                        if (turnerState != TurnerState.FORWARD) {
                            turnerTimer.reset();
                            gripper.turnerSetPosition2(); //fwd
                            liftState = LiftState.LIFT_TURNER_FRONT;
                        }
                        else {
                            liftState = LiftState.LIFT_HOLD;
                        }
                    }
                    break;

                case LIFT_LOW:
                    //action: check if height is within half inch of target, if not wait until it is
                    if (Math.abs(slideTrainer.getSlidePos() - slideTrainer.SLIDE_LEVEL_3) < 0.5) {

                        heightLow = true;

                        if (turnerState != TurnerState.FORWARD) {
                            turnerTimer.reset();
                            gripper.turnerSetPosition2(); //fwd
                            liftState = LiftState.LIFT_TURNER_FRONT;
                        }
                        else {
                            liftState = LiftState.LIFT_HOLD;
                        }
                    }
                    break;

                case LIFT_TURNER_FRONT:
                    //action: check if enough time has passed to turn
                    if (turnerTimer.seconds() >= 0.2) {
                        turnerState = TurnerState.FORWARD;
                        liftState = LiftState.LIFT_HOLD;
                    }
                    break;

                case LIFT_HOLD:
                    if (gamepad2.left_trigger > 0.25 || gamepad2.right_trigger > 0.25) {
                        turnerTimer.reset();
                        gripper.turnerSetPosition1(); //back
                        liftState = LiftState.LIFT_TURNER_BACK;
                    }
                    else if (gamepad2.dpad_right) {
                        slideTrainer.setSlideLevel5();
                        liftState = LiftState.LIFT_HIGH;
                        heightLow = false;
                    }
                    else if (gamepad2.dpad_up) {
                        slideTrainer.setSlideLevel4();
                        liftState = LiftState.LIFT_MED;
                        heightLow = false;
                    }
                    else if (gamepad2.dpad_left) {
                        slideTrainer.setSlideLevel3();
                        liftState = LiftState.LIFT_LOW;
                    }
                    break;

                case LIFT_TURNER_BACK:
                    double timeValue = 0.2;
                    //action: check if enough time has passed to turn
                    if (heightLow == true) {
                        timeValue = 1;
                    }
                    if (turnerTimer.seconds() >= timeValue) {
                        turnerState = TurnerState.BACK;
                        slideTrainer.setSlideLevel1(); //this function already has lower power on way down
                        liftState = LiftState.LIFT_GET_CONE;
                        heightLow = false;
                    }
                    break;

                case LIFT_GET_CONE:
                    //action: check if the lift is within half inch of fully lowered
                    if (Math.abs(slideTrainer.getSlidePos() - slideTrainer.SLIDE_LEVEL_1) < 0.5) {
                        slideTrainer.slidemotorback.setPower(0);
                        slideTrainer.slidemotorfront.setPower(0);
                        liftState = LiftState.LIFT_IDLE;
                    }
                    break;

            }
             */

            if (gamepad2.back) {

            }
        }
    }

    void debounce(long debounceTime) {
        try {
            Thread.sleep(debounceTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
}