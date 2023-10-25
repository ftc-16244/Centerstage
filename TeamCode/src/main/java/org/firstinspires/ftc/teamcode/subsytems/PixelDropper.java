package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config // this is so the dashboard will pick up variables
public class PixelDropper {

    //Define Hardware Objects
    public Servo pixelDropperServo = null;
    public VoltageSensor    voltSensor         = null;

    //Constants for dropper
    public static final double      DROPPER_OPEN       = 0.25; // not dropped
    public static final double      DROPPER_CLOSED      = 0.64 ; // pixel dropped

    Telemetry       telemetry;
    LinearOpMode    opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    /// constructor with opmode passed in
    public PixelDropper(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void init(HardwareMap hwMap)  {
        // Initialize the dropper
        pixelDropperServo = hwMap.get(Servo.class,"pixelDropperServo");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////

    public void dropperInitTeleop(){
        pixelDropperServo.setPosition(DROPPER_OPEN);
    }

    public void dropperInitAuto(){
        pixelDropperServo.setPosition(DROPPER_CLOSED);
    }
    public void dropperClosed(){
        pixelDropperServo.setPosition(DROPPER_CLOSED);

    }
    public void dropperOpen(){
        pixelDropperServo.setPosition(DROPPER_OPEN);
    }

}
