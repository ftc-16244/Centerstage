package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.Drone;
import org.firstinspires.ftc.teamcode.subsytems.Felipe2;
import org.firstinspires.ftc.teamcode.subsytems.Juan;


@Config
@TeleOp(group = "Teleop")
public class ImplementTest extends LinearOpMode {
// test
    // random comment


    Felipe2 felipe2 = new Felipe2(this);
    Juan juan = new Juan(this);

    Drone drone  = new Drone(this);


    private ElapsedTime teleopTimer = new ElapsedTime();
    private double TELEOP_TIME_OUT = 130;


    FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {


        // set up local variables

        double slidePosition;
        double speedFactor = 1.0;
        double expo = 3; // has to be 1 or 3

        // set up Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Initialize the sub systems. Note the init method is inside the subsystem class
        felipe2.init(hardwareMap);
        felipe2.gripperWideOpen();
        felipe2.setAnglerLoad();
        felipe2.setSlideLevel_0();
        //felipe2.rotateToTargetAngle(145,1,0.5);

        // Telemetry

        telemetry.addData("Lift State", null);
        //telemetry.addData("Angler State", anglerState);
        //telemetry.addData("Height Low", heightLow);
        //dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////

        waitForStart();

        while (!isStopRequested() && teleopTimer.time() < TELEOP_TIME_OUT) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            Math.pow(-gamepad1.left_stick_y, expo) * speedFactor,
                            Math.pow(-gamepad1.left_stick_x, expo) * speedFactor,
                            Math.pow(-gamepad1.right_stick_x, expo) * speedFactor
                    )
            );

            ////////////////////////////////////////////////////////////////////////////////
                                             //GAMEPAD 1//
            ///////////////////////////////////////////////////////////////////////////////
            // Button names and positions
            //        y
            //      x   b
            //        a
            if (gamepad1.a) {
                felipe2.setTurnerLoad();
                felipe2.setSlideLevel_0();
                
            }
            if (gamepad2.y) {
                felipe2.setTurnerDeploy();

            }
            if (gamepad1.dpad_down) {
                felipe2.setSlideLevel_0();
            }

            if (gamepad1.dpad_left) {
                felipe2.setSlideRow_1();
            }

            if (gamepad1.dpad_up) {
            }

            if (gamepad1.dpad_right) {

            }
            if (gamepad1.left_trigger > 0.25) {
                felipe2.gripperClosed();
                // gp1lefttrigger here
            }

            if (gamepad1.right_trigger > 0.25) {
                felipe2.gripperOpen();
                // gp1righttrigger here
            }

            if (gamepad1.right_bumper){
                //felipe2.setTurnerDeploy(); // where to put this
                felipe2.gripperRightOpen();
                sleep(100);

            }

            if (gamepad1.left_bumper){
                //felipe2.setTurnerLoad(); // need to move this
                felipe2.gripperLeftOpen();
                sleep(100);
            }


            ////////////////////////////////////////////////////////////////////////////////
                                             //GAMEPAD 2//
            ///////////////////////////////////////////////////////////////////////////////
            // Button names and positions
            //        y
            //      x   b
            //        a

            if (gamepad2.a) {
                juan.climb();
                sleep(500);
            }
            if (gamepad2.y) {
                juan.prepForClimb();
                //gp2y();
                sleep(500);
            }
            if (gamepad2.b) {
                //gp2b();
                juan.reset();
            }
            if (gamepad2.back) {
                felipe2.slideMechanicalReset();
            }
            if (gamepad2.x) {
                drone.setDroneFly(); // move servo to let drone go
                sleep(50); // pause to make sure servo moves

            if (gamepad2.dpad_down) {
                felipe2.setSlideLevel_0();
            }

            if (gamepad2.dpad_right) {
                felipe2.setSlideRow_2();

            }

            if (gamepad2.dpad_up) {
                //felipe2.setSlideRow_4(); // don't use until wires are fixed
            }

            if (gamepad2.dpad_left) {

            }
            if (gamepad2.left_trigger > 0.25) {
                felipe2.setAnglerDeploy();
                sleep(50);

            }

            if (gamepad2.right_trigger > 0.25) {
                felipe2.setAnglerLoad();
                sleep(50);
                felipe2.gripperOpen();
                sleep(50);
            }

            if (gamepad2.right_bumper){

            }
            if (gamepad2.left_bumper){

            }

        }
            }


            }}