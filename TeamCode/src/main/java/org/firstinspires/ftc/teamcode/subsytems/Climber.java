package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
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
    private static final double LIFT_HEIGHT_CORRECTION_FACTOR = 1.13;
    private static final double TICKS_PER_MOTOR_REV = 384.5; // goBilda 435  //312 RPM  537.7
    private static final double PULLEY_DIA = 40; // milimeters
    private static final double LIFT_DISTANCE_PER_REV = PULLEY_DIA * Math.PI / (25.4 * LIFT_HEIGHT_CORRECTION_FACTOR);
    private static final double TICKS_PER_LIFT_IN = TICKS_PER_MOTOR_REV / LIFT_DISTANCE_PER_REV;

    private static final int climberDeployed = 385;
    private static final int climberStowed = 0;
    private static final int winchDeployed = 3850;
    private static final int winchStowed = 0;

    public double targetHeight;


    public void init(HardwareMap hardwareMap) {
        climber = hardwareMap.get(DcMotorEx.class, "climberMotor");
        winch = hardwareMap.get(DcMotorEx.class, "winchMotor");

        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setCurrentAlert(9, CurrentUnit.AMPS); // Stall rating is 9.2
    }




    public void liftToTargetHeight(double height, double timeoutS, double SLIDELIFTSPEED) {

        int newTargetHeight;


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int) (height * TICKS_PER_LIFT_IN);
            // Set the target now that is has been calculated
            climber.setTargetPosition(newTargetHeight);
            // Turn On RUN_TO_POSITION
            climber.setPower(Math.abs(SLIDELIFTSPEED));
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
