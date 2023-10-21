package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {
    public DcMotorEx climber;
    public DcMotorEx winch;

    private static final int climberDeployed = 385;
    private static final int climberStowed = 0;
    private static final int winchDeployed = 3850;
    private static final int winchStowed = 0;

    public void init(HardwareMap hardwareMap) {
        climber = hardwareMap.get(DcMotorEx.class,"climberMotor");
        winch = hardwareMap.get(DcMotorEx.class, "winchMotor");

        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setCurrentAlert(9, CurrentUnit.AMPS); // Stall rating is 9.2
    }
}
