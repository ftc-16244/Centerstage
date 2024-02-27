package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Juan {
    public DcMotorEx climberRight;
    public DcMotorEx climberLeft;
    public ServoImplEx manarsFootLeft;
    public ServoImplEx manarsFootRight;

    public VoltageSensor voltSensor = null;
    Telemetry telemetry;
    LinearOpMode opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    //================================ DUAL CLIMBER==========================================================
    private static final int CLIMB_DEPLOY_TICKS = 2100; // a little less than the meet 3 robot
    private static final int CLIMB_STOW_TICKS = 25;
    private static final double HOOK_DEPLOY = 0.5;
    private static final double HOOK_STOW_LEFT = 0.15;
    private static final double HOOK_STOW_RIGHT = 0.85;

    private static final double CLIMB_POWER = 1.0;

    // Constructor
    public Juan(LinearOpMode opmode) { 
        this.opmode = opmode;

    }

    public void init(HardwareMap hardwareMap) {
        climberRight = hardwareMap.get(DcMotorEx.class, "climberRight"); // EXP Hub Motor Port
        climberLeft = hardwareMap.get(DcMotorEx.class, "climberLeft");// EXP hub mtor port
        manarsFootLeft = (ServoImplEx) hardwareMap.get(Servo.class, "manarsLeftFoot"); // CH servo port 0
        manarsFootRight = (ServoImplEx) hardwareMap.get(Servo.class, "manarsRightFoot");// CH servo port 2

        // set directions
        climberRight.setDirection(DcMotorEx.Direction.REVERSE);
        climberLeft.setDirection(DcMotorEx.Direction.REVERSE);

        climberLeft.setCurrentAlert(6, CurrentUnit.AMPS); // Stall current is 9.2A
        climberRight.setCurrentAlert(6, CurrentUnit.AMPS);

        climberLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climberRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        climberLeft.setPower(-0.3);
        climberRight.setPower(-0.3);

        runtime.reset();
        while (runtime.seconds() < 5.0) {
            //Time wasting loop so climber can retract. Loop ends when time expires
        }
        climberLeft.setPower(0);
        climberRight.setPower(0);

        // set to encoder operation
        climberRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        climberLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        manarsFootLeft.setPwmEnable();
        manarsFootRight.setPwmEnable();

        reset();
    }

    public void prepForClimb() {
        manarsFootLeft.setPwmEnable();
        manarsFootRight.setPwmEnable();

        climberLeft.setTargetPosition(CLIMB_DEPLOY_TICKS);
        climberRight.setTargetPosition(CLIMB_DEPLOY_TICKS);

        climberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        climberLeft.setPower(CLIMB_POWER);
        climberRight.setPower(CLIMB_POWER);

        manarsFootLeft.setPosition(HOOK_DEPLOY - 0.01); // to make the hook straight up
        manarsFootRight.setPosition(HOOK_DEPLOY);
    }
    public void climb() {
        manarsFootLeft.setPwmDisable();
        manarsFootRight.setPwmDisable();

        climberLeft.setTargetPosition(CLIMB_STOW_TICKS);
        climberRight.setTargetPosition(CLIMB_STOW_TICKS);

        climberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        climberLeft.setPower(CLIMB_POWER);
        climberRight.setPower(CLIMB_POWER);

    }
    public void reset() {
        manarsFootLeft.setPwmEnable();
        manarsFootRight.setPwmEnable();

        climberLeft.setTargetPosition(CLIMB_STOW_TICKS);
        climberRight.setTargetPosition(CLIMB_STOW_TICKS);

        climberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        manarsFootLeft.setPosition(HOOK_STOW_LEFT);
        manarsFootRight.setPosition(HOOK_STOW_RIGHT);

    }

}
