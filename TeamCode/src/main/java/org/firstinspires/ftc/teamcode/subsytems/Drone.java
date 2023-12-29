package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drone {
    private final LinearOpMode opmode;
    public Servo teachers;
    LinearOpMode opMode;
    ElapsedTime runtime = new ElapsedTime();
    public static final double DRONE_GROUNDED = 0.75; // drone lock position

    public static final double DRONE_FLY = 0.3;  // release drone position
    public void setDroneGrounded() {
        teachers.setPosition(DRONE_GROUNDED);
    }
    public void setDroneFly() {
        teachers.setPosition(DRONE_FLY);
    }
    public Drone(LinearOpMode opmode) {
        this.opmode = opmode;

    }

    public void init(HardwareMap hwmap) {
        teachers = hwmap.get(Servo.class, "droneServo");
    }

}
