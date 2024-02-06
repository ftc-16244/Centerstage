package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;



public class Juan {
    public DcMotorEx climberRight;
    public DcMotorEx climberLeft;
    public Servo manarsFootLeft;
    public Servo manarsFootRight;

    public VoltageSensor voltSensor = null;


    Telemetry telemetry;
    LinearOpMode opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    //================================ DUAL CLIMBER==========================================================
    private static final int CLIMB_DEPLOY_TICKS = 2100; // a little less than the meet 3 robot
    private static final int CLIMB_STOW_TICKS = 25;
    private static final double HOOK_DEPLOY = 0.5;
    private static final double HOOK_STOW = 0.02;


    private static final double CLIMB_POWER = 1.0;

    // Constructor
    public Juan(LinearOpMode opmode) {
        this.opmode = opmode;

    }



    public void init(HardwareMap hardwareMap) { 
        climberRight = hardwareMap.get(DcMotorEx.class, "climberRightMotor"); // EXP Hub Motor Port
        climberLeft = hardwareMap.get(DcMotorEx.class, "climberLeftMotor");// EXP hub mtor port
        manarsFootLeft = hardwareMap.get(Servo.class, "manarsLeftFoot"); // CH servo port 0
        manarsFootRight = hardwareMap.get(Servo.class, "manarsRightFoot");// CH servo port 2


        // set directions
        climberRight.setDirection(DcMotorEx.Direction.REVERSE);
        climberLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // set to encoder operation
        climberRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void prepForClimb() {
        climberLeft.setTargetPosition(CLIMB_DEPLOY_TICKS);
        climberRight.setTargetPosition(CLIMB_DEPLOY_TICKS);
        climberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberLeft.setPower(CLIMB_POWER);
        climberRight.setPower(CLIMB_POWER);
        manarsFootLeft.setPosition(HOOK_DEPLOY);
        manarsFootRight.setPosition(HOOK_DEPLOY);
    }
    public void climb() {
        climberLeft.setTargetPosition(CLIMB_STOW_TICKS);
        climberRight.setTargetPosition(CLIMB_STOW_TICKS);
        climberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberLeft.setPower(CLIMB_POWER);
        climberRight.setPower(CLIMB_POWER);

    }
    public void reset() {
        climberLeft.setTargetPosition(CLIMB_STOW_TICKS);
        climberRight.setTargetPosition(CLIMB_STOW_TICKS);
        climberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberLeft.setPower(HOOK_STOW);
        climberRight.setPower(HOOK_STOW);

    }

}
