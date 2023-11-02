package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {
    public DcMotorEx climber;
    public DcMotorEx winch;
    public VoltageSensor voltSensor = null;


    Telemetry telemetry;
    LinearOpMode opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    //================================ CLIMBER==========================================================

    // Climber Constants - Motor and mechanism
    public static double CLIMB_SPEED = 0.40; // dont want this to be too fast
    public static double CLIMB_DRIVE_RATIO = 2; // drive sprocket is 14 teeth and teh driven is 28 teeth
    private static final double CLIMBER_TICKS_PER_MOTOR_REV = 537.7; // goBilda 435 -->384.5 //312--> RPM  537.7

    // Climber Constants - Arm Angles

    private static final double CLIMBER_DEPLOY_ANGLE = 70; // degrees - change this variable to fine tune.
    private static final double CLIMBER_HANGING_ANGLE = 10; // degrees - change this variable to fine tune.
    private static final double CLIMBER_STOWED_ANGLE = 0; // degrees - change this variable to fine tune.
    private static final double CLIMBER_STOW_SPEED = 0.1;



    // Climber Constants - Convert to ticks

    //private static final int CLIMBER_DEPLOY_TICKS 	= CLIMB_DRIVE_RATIO*CLIMBER_TICKS_PER_MOTOR_REV*CLIMBER_DEPLOY_ANGLE/360;  // units are ticks
    //private static final int CLIMBER_HANGING_TICKS 	= CLIMB_DRIVE_RATIO*CLIMBER_TICKS_PER_MOTOR_REV*CLIMBER_HANGING_ANGLE/360;  // units are ticks
    //private static final int CLIMBER_STOWED_TICKS 	= CLIMB_DRIVE_RATIO*CLIMBER_TICKS_PER_MOTOR_REV*CLIMBER_HANGING_ANGLE/360;  // units are ticks should be zero


    //================================ WINCH==========================================================

    // Winch Constants - Motor and mechanism

    public static double WINCH_SPEED = 1.0; // go as fast as possible
    // the drive and driven sprockets are both 14 teeth so no need for a gear ratio here.
    private static final double WINCH_TICKS_PER_MOTOR_REV = 1425.1; // 117 RPM goBilda Motor
    private static final double WINCH_PULLEY_DIA 	= 40; // milimeters

    // Winch Constants - String Distances

    private static double WINCH_DEPLOY_DISTANCE 	= 16.5 ; // inches (need to test to get correct value here)
    private static double WINCH_HANGING_DISTANCE 	= 5.5; // inches (need to test to get correct value here)
    private static double WINCH_STOWED_DISTANCE 	= 0; // inches - should usually be zero. Can change if needed


    public double targetClimbAngle; // local variable
    public double targetWinchDistance; // local variable

    // Constructor
    public Climber(LinearOpMode opmode) {
        this.opmode = opmode;

    }



    public void init(HardwareMap hardwareMap) {
        climber = hardwareMap.get(DcMotorEx.class, "climberMotor");
        winch = hardwareMap.get(DcMotorEx.class, "winchMotor");
        // set directions
        climber.setDirection(DcMotorEx.Direction.REVERSE);
        winch.setDirection(DcMotorEx.Direction.FORWARD);
        // set to encoder operation
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // stop and rest encoders
        //climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // current alert
        winch.setCurrentAlert(9, CurrentUnit.AMPS); // Stall rating is 9.2
    }




    public void setToTargetClimbAngle(double climber_angle, double timeoutS) {

        int newTargetClimbAngle;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift angle in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetClimbAngle = (int) (CLIMB_DRIVE_RATIO*CLIMBER_TICKS_PER_MOTOR_REV*climber_angle/360); // note newTargetClimbAngle is in ticks not degrees
            // Set the target now that is has been calculated
            climber.setTargetPosition(newTargetClimbAngle);
            // Turn On RUN_TO_POSITION
            climber.setPower(Math.abs(CLIMB_SPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            climber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (opmode.opModeIsActive() &&
            //       (runtime.seconds() < timeoutS) && climber.isBusy()) {
            // holds up execution to let the climber go up to the right place - leave commented out for now

            // }


        }

    }

    public void setToWinchSpoolOut(double spool_length, double timeoutS) {

        int newTargetSpoolLength;


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift angle in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetSpoolLength = (int) (spool_length*WINCH_TICKS_PER_MOTOR_REV*25.4/(3.14*WINCH_PULLEY_DIA)); // note newTargetClimbAngle is in ticks not degrees
            // Set the target now that is has been calculated
            winch.setTargetPosition(newTargetSpoolLength);
            // Turn On RUN_TO_POSITION
            winch.setPower(Math.abs(CLIMB_SPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opmode.opModeIsActive() &&
                   (runtime.seconds() < timeoutS) && climber.isBusy()) {
            //holds up execution to let the climber go up to the right place - leave commented out for now

             }


        }

    }

    public void climberDeploy(){
        setToTargetClimbAngle(CLIMBER_DEPLOY_ANGLE, 10);

    }

    public void climberHang(){
        setToTargetClimbAngle(CLIMBER_HANGING_ANGLE, 10);
    }

    public void climberStow(){
        setToTargetClimbAngle(CLIMBER_STOWED_ANGLE, 3);
    }

    public void winchDeploy(){
        setToWinchSpoolOut(WINCH_DEPLOY_DISTANCE, 3);
    }
    public void winchHang(){
        setToWinchSpoolOut(WINCH_HANGING_DISTANCE, 3);
    }
    public void winchStow(){
        setToWinchSpoolOut(WINCH_STOWED_DISTANCE, 3);
    }

}