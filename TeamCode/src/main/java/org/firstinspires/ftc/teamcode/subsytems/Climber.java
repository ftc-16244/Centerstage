package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {
    private final LinearOpMode opmode;
    public DcMotorEx hangman;
    public ServoImplEx manarsFoot;
    public ElapsedTime runtime = new ElapsedTime();
    private static final int CLIMB_DEPLOY_TICKS = 2450;
    private static final int CLIMB_STOW_TICKS = 25;
    private static final double HOOK_DEPLOY = 0.5;
    private static final double HOOK_STOW = 0.02;

    private static final double CLIMB_POWER = 1.0;
    public Climber(LinearOpMode opmode) {
        this.opmode = opmode;

    }
    public void init(HardwareMap hwMap) {
        hangman = hwMap.get(DcMotorEx.class, "hangman");
        manarsFoot = (ServoImplEx) hwMap.get(Servo.class, "manarsFoot");
        hangman.setDirection(DcMotorSimple.Direction.FORWARD);
        hangman.setPower(1.0);

        hangman.setCurrentAlert(6, CurrentUnit.AMPS); // Stall current is 9.2A

        hangman.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangman.setPower(-0.2);
        runtime.reset();
        while (runtime.seconds() < 2.0) {
            //Time wasting loop so climber can retract. Loop ends when time expires
        }
        hangman.setPower(0);

        hangman.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangman.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void prepForClimb() {
        manarsFoot.setPwmEnable();
        hangman.setTargetPosition(CLIMB_DEPLOY_TICKS);
        hangman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangman.setPower(CLIMB_POWER);
        manarsFoot.setPosition(HOOK_DEPLOY);
    }
    public void climb() {
        hangman.setTargetPosition(CLIMB_STOW_TICKS);
        manarsFoot.setPwmDisable();
        hangman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangman.setPower(CLIMB_POWER);
    }
    public void reset() {
        manarsFoot.setPwmEnable();
        hangman.setTargetPosition(CLIMB_STOW_TICKS);
        hangman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        manarsFoot.setPosition(HOOK_STOW);
    }
}
