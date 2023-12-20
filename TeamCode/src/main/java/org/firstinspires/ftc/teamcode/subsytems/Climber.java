package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {
    public DcMotorEx hangman;
    public Servo manarsFoot;
    private static final int CLIMB_DEPLOY_TICKS = 2450;
    private static final int CLIMB_STOW_TICKS = 25;
    private static final double HOOK_DEPLOY = 0.5;
    private static final double HOOK_STOW = 0.75;
    public void init(HardwareMap hwMap) {
        hangman = hwMap.get(DcMotorEx.class, "hangman");
        manarsFoot = hwMap.get(Servo.class, "manarsFoot");
        hangman.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangman.setDirection(DcMotorSimple.Direction.FORWARD);
        hangman.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangman.setPower(1.0);

        hangman.setCurrentAlert(6, CurrentUnit.AMPS); // Stall current is 9.2A
    }
    public void prepForClimb() {
        hangman.setTargetPosition(CLIMB_DEPLOY_TICKS);
        hangman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        manarsFoot.setPosition(HOOK_DEPLOY);
    }
    public void climb() {
        hangman.setTargetPosition(CLIMB_STOW_TICKS);
        hangman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void reset() {
        hangman.setTargetPosition(CLIMB_STOW_TICKS);
        hangman.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        manarsFoot.setPosition(HOOK_STOW);
    }
}
