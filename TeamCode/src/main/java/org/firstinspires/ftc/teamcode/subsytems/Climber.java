package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {
    public DcMotorEx climber;
    public DcMotorEx winch;
    public VoltageSensor voltSensor = null;


    Telemetry telemetry;
    LinearOpMode opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    public static double LIFTSPEED = 0.70; //
    private static final double TICKS_PER_MOTOR_REV = 384.5; // goBilda 435  //312 RPM  537.7
    private static final double PULLEY_DIA = 40; // milimeters
    private static final double PULLEY_DISTANCE_PER_ANGLER_DEGREE = (PULLEY_DIA * Math.PI) / 1;

    private static final int climberDeployed = 385;
    private static final int climberStowed = 0;
    private static final int winchDeployed = 3850;
    private static final int winchStowed = 0;

    public double targetHeight;
    public double ticks;

    public Climber(LinearOpMode opmode) {
        this.opmode = opmode;

    }

    public void init(HardwareMap hardwareMap) {
        climber = hardwareMap.get(DcMotorEx.class, "climberMotor");
        winch = hardwareMap.get(DcMotorEx.class, "winchMotor");

        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setCurrentAlert(9, CurrentUnit.AMPS); // Stall rating is 9.2
    }

public void climberInitTeleop(){
        climber.setTargetPosition(climberStowed);
        winch.setTargetPosition(winchStowed);
    }


    public void climberGoToPosition(int climberPosition, double speed) {
        if (opmode.opModeIsActive()) {
            climber.setTargetPosition(climberPosition);
            climber.setPower(Math.abs(speed));
            // reset the timeout time and start motion.
            runtime.reset();
            climber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (opmode.opModeIsActive() &&
            //       (runtime.seconds() < timeoutS) && slidemotorback.isBusy() && slidemotorfront.isBusy()) {
            // holds up execution to let the slide go up to the right place

            // }
        }
    }
}
