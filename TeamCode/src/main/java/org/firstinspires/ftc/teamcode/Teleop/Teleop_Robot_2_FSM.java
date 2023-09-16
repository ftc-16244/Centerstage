package org.firstinspires.ftc.teamcode.Teleop2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.SlideTrainerState;
import org.firstinspires.ftc.teamcode.Subsystem2.Gripper2;
import org.firstinspires.ftc.teamcode.Subsystem2.Slide_Trainer2;
import org.firstinspires.ftc.teamcode.Subsystem2.TrackingWheelLifters;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@TeleOp(group = "Teleop")
@Disabled

public class Teleop_Robot_2_FSM extends LinearOpMode {


    ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime teleopTimer = new ElapsedTime();
    private double TELEOP_TIME_OUT = 130;

    FtcDashboard dashboard;

    Slide_Trainer2 slideTrainer = new Slide_Trainer2(this);
    SlideTrainerState slideTrainerState = SlideTrainerState.UNKNOWN;

    Gripper2 gripper = new Gripper2(this);

    TrackingWheelLifters trkWhlLifters = new TrackingWheelLifters(this);

    private TurnerState turnerState = TurnerState.DISABLED; //
    private TurnerPosition turnerPos = TurnerPosition.BACK; //
    private  DriverCommandLevel driverCommandLevel= DriverCommandLevel. GET_CONE_GROUND; //
    private SlideHeight slideHeight = SlideHeight.SUBSTAION_LOAD;

    //ENUMS
    enum TurnerState {
        ENABLED,
        DISABLED}

    enum TurnerPosition {
        FORWARD,
        BACK}

    enum DriverCommandLevel{
        GET_CONE_GROUND,
        GROUND_JUNCTION,
        LOW,
        MED,
        HIGH}
    enum SlideHeight{
        SUBSTAION_LOAD,
        GROUND_JUMCTION,
        LOW_JUMCTION,
        MED_JUMCTION,
        HIGH_JUMCTION}

    @Override
    public void runOpMode() throws InterruptedException {


        // set up local variables
        double  slidePosition;
        double  speedFactor = 1.0;
        double expo =   3; // has to be 1 or 3

        // set up Mecanum Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the sub systems. Note the init method is inside the subsystem class
        slideTrainer.init(hardwareMap);
        gripper.init(hardwareMap);
        trkWhlLifters.init(hardwareMap);


        // Move servos to start postion. Grippers open and track wheels up (for teleop)

        gripper.turnerSetPosition1();//back
        gripper.gripperOpen(); // for teleop start with the gripper open. for Auto is needs to be closed to hold the cone
        trkWhlLifters.trkWhlsUp(); // lift up for teleop put down for auto

        // Telemetry
        telemetry.addData("Lift State", slideTrainerState);
        telemetry.addData("Turner State", turnerState);
        telemetry.addData("Turner Position", turnerPos);
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

          // get slide height each time thhrouhg the loop to decide if turner can be used or not
          // this returns height in inches
          slidePosition = slideTrainer.getSlidePos();


          if (slidePosition <= slideTrainer.TURNER_MIN_HEIGHT_2_ROTATE){
             turnerState = TurnerState.DISABLED;

          }
          else { turnerState = TurnerState.ENABLED;

          }


            if (gamepad1.dpad_right) {
                slideTrainer.setSlideCone4();

            }

            if (gamepad1.dpad_up) {
                //slideTrainer.setSlideCone5();
                driverCommandLevel = DriverCommandLevel.HIGH; // change the state based on button press


            }

            if (gamepad1.dpad_down) {
                slideTrainer.setSlideCone3();


            }

            if (gamepad1.dpad_left) {
                slideTrainer.setSlideCone2();
            }

            if (gamepad1.back) {
                slideTrainer.slideMechanicalReset();
                slideTrainer.targetHeight = 0;
            }
            if (gamepad1.left_trigger > 0.25) {

                gripper.TopArmOpen(); // top arm of gripper
                gripper.gripperOpen(); //cam type gippers

            }

            if (gamepad1.right_trigger > 0.25) {
                gripper.topArmClosed();
                sleep(gripper.GRIPPER_DELAY); // add slight delay to allow top arm to get to cone first

                gripper.gripperClosed();
            }
//// GAMEPAD #2/////////////////////////
            if (gamepad2.a  &&  turnerState == TurnerState.ENABLED) {
                gripper.turnerSetPosition1();
                turnerPos= TurnerPosition.BACK; // sets state to back after requesting a servo move to the back
            }

            if (gamepad2.y  &&  turnerState == TurnerState.ENABLED) {
                gripper.turnerSetPosition2();
                turnerPos= TurnerPosition.FORWARD; //
            }


            if (gamepad2.dpad_left) {
                slideTrainer.setSlideLevel3();
            }

            if (gamepad2.dpad_up) {
                slideTrainer.setSlideLevel4();
            }

            if (gamepad2.dpad_right) {
                slideTrainer.setSlideLevel5();
            }
            // this makes sure the gripper is in the back position before lowering. Otherwise ir hits the front of the robot chassis
            if (gamepad2.left_trigger > 0.25 && turnerPos == TurnerPosition.BACK) {
                slideTrainer.setSlideLevel2();
            }

            if (gamepad2.right_trigger > 0.25 && turnerPos == TurnerPosition.BACK) {
                slideTrainer.setSlideLevel1();
            }

            if (gamepad2.back) {
                slideTrainer.slideMechanicalReset();
                slideTrainer.targetHeight = 0;
            }
            ///////////////Switch Case to process driver commands
            switch(driverCommandLevel){
                case HIGH:
                    if (driverCommandLevel != DriverCommandLevel.HIGH) {
                        driverCommandLevel = DriverCommandLevel.HIGH;
                        slideTrainer.setSlideLevel5();
                       // once slide is at a desired level, change turner state to Forward
                        // but servos don't like being in these loops so have to put a short wait somehere.
                    }

                    break;
                case MED:
                    if (driverCommandLevel != DriverCommandLevel.MED) {
                        driverCommandLevel = DriverCommandLevel.MED;
                    }
                    break;
            }

        }//end of the while loop


    }

    void debounce(long debounceTime) {
        try {
            Thread.sleep(debounceTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
}