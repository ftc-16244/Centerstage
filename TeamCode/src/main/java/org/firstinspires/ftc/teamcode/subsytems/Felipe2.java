package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

@Config // this is so the dashboard will pick up variables
public class Felipe2 {

    //Define Hardware Objects
    public Servo            angler             = null;
    public Servo            gripperRight       = null;
    public Servo            gripperLeft        = null;
    public Servo            armWheel            = null;
    public VoltageSensor    voltSensor         = null;
    public Servo            SlideWheelServo    = null;

    public  DcMotorEx       extendMotor;  // config name is "slideMotor"
    public DcMotorEx        turnerMotor; // config name is "turnerMotor"

    //Constants for gripper
    //larer numbers are more clockwise

    public static final double      GRIPPER_LEFT_CLOSED      = 0.45; //to close more, decrease
    public static final double      GRIPPER_LEFT_OPEN        = GRIPPER_LEFT_CLOSED + 0.15; //to open more, increase
    public static final double      GRIPPER_LEFT_WIDE_OPEN   = GRIPPER_LEFT_CLOSED +.32;

    public static final double      GRIPPER_RIGHT_CLOSED     = 0.48;// to close more, increase
    public static final double      GRIPPER_RIGHT_OPEN       =  GRIPPER_RIGHT_CLOSED -0.15; // too open more, decrease
    public static final double      GRIPPER_RIGHT_WIDE_OPEN  = GRIPPER_RIGHT_CLOSED -0.32; // not gripped

    //Constants for slidewheel

    public static final double      ARM_WHEEL_PIXEL_5        = 1; //stowed position
    public static final double      ARM_WHEEL_PIXEL_4        = 1; //to open more, increase
    public static final double      ARM_WHEEL_STOW        = 1; //pick up 4th and 5th pixel on stack


    //Constants for angler
    //NOTE: lower values make the angler go higher, higher values make it go lower
    public static final double      ANGLER_CARRY       = 0.44;// load and moving the pixel
    public static final double      ANGLER_DEPLOY      = 0.45; // deposit the pixel
    public static final double      ANGLER_LOAD      = 0.493; // Loading the pixel

    Telemetry       telemetry;
    LinearOpMode    opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public  static double           SLIDESPEED                  = 1.00; // full speed
    public  static double           SLIDESPEEDSLOWER            = 0.5; //half speed
    public static  double           SLIDERESETSPEED             = -0.2; // only used to retract and reset slide encoder
    public static final double      SLIDE_LEVEL_0               = 0.25;// Extension fully retracted but not to mechanical stop
    public static final double      SLIDE_LEVEL_ROW_1           = 3; // First yellow autp high accuracy
    public static final double      SLIDE_LEVEL_ROW_2           = 6; // Second row of pixels
    public static final double      SLIDE_LEVEL_ROW_4           = 12; // reserve
    public static final double      SLIDE_LEVEL_ROW_6           = 18; // auto drop pixel in right spot

    public static final double      SLIDE_REACH_1         = 2; // Used to reach out horizontally to drop purple ot get white
    public static final double      SLIDE_REACH_2         = 4; // Yellow pixel gentle drop (first yellow)
    public static final double      SLIDE_REACH_3          = 6;
    public static final double      SLIDE_REACH_4         = 10;

    private static final double     SLIDE_HEIGHT_CORRECTION_FACTOR   =   1.00;
    private static final double     TICKS_PER_MOTOR_REV_SLIDE             = 145.1 * 2; // goBilda 1150 RPM MOTOR and 2:1 Bevel Gear

    /////////////
    // TURNER ARM ROTATOR

    private static final double     TICKS_PER_MOTOR_REV_TURNER       = 2850.2; // (1425.1 * 2) goBilda 435  //312 RPM  537.7
    private static final double     PULLEY_DIA                      = 38.2; // milimeters
    private static final double     SLIDE_DISTANCE_PER_REV           = PULLEY_DIA * Math.PI / (25.4 * SLIDE_HEIGHT_CORRECTION_FACTOR);
    private static final double     TICKS_PER_SLIDE_IN               = TICKS_PER_MOTOR_REV_SLIDE / SLIDE_DISTANCE_PER_REV;
    private static final double     TICKS_PER_TURNER_DEGREE             = TICKS_PER_MOTOR_REV_TURNER / 360;

    private static final double     TURNER_SPEED = 0.35;
    public static final double     TURNER_PRECISE_SPEED = 0.20; // used to help the arm float with arm wheel


    //Constants for Turner
    public static final double      TURNER_DEPLOY_ANGLE =  145; // deposit the pixel
    public static final double      TURNER_LOAD_ANGLE      = 0; // Loading the pixel
    public static final double     PIXEL_4_ANGLE =10; // pick up 4th and possibly 5th pixel from the mat.
    public static final double     PIXEL_5_ANGLE =11; // pick up top or 5th pixel only from white stack
    public double  targetHeight;
    public double  targetAngle;


    /// constructor with opmode passed in
    public Felipe2(LinearOpMode opmode) {
        this.opmode = opmode;

    }

    public void init(HardwareMap hwMap)  {

        //voltSensor = hwMap.voltageSensor.get("Expansion Hub 2");

        // Initialize angler
        angler = hwMap.get(Servo.class,"anglerServo"); // Exp Hub port 0
        //setanglerCarry();

        // Initialize the gripper
        gripperRight = hwMap.get(Servo.class,"gripperRightServo"); //Exp Hub port 4
        gripperLeft = hwMap.get(Servo.class,"gripperLeftServo"); //Exp Hub port 2

        armWheel = hwMap.get(Servo.class,"armWheelServo"); //Exp Hub port 3

        // Initialize the lift motor
        extendMotor = hwMap.get(DcMotorEx.class,"liftMotor");
        extendMotor.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidfOrig = extendMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        //PIDFCoefficients pidSlide_New = new PIDFCoefficients(SLIDE_NEW_P, SLIDE_NEW_I, SLIDE_NEW_D, SLIDE_NEW_F);
        //slidemotorback.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);
        //slidemotorfront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);
        // re-read coefficients and verify change.
        //PIDFCoefficients pidModifiedback = slidemotorback.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //PIDFCoefficients pidModifiedfront = slidemotorfront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //slidemotorback.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);
        //slidemotorfront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);

        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);




    }

    //Angler methods
    public void setAnglerLoad() {
        angler.setPosition(ANGLER_LOAD);//fwd
    }
    public void setAnglerCarry() {
        angler.setPosition(ANGLER_CARRY); // back
    }
    public void setAnglerDeploy() {
        angler.setPosition(ANGLER_DEPLOY);//fwd
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////

    public void gripperWideOpen(){
        gripperRight.setPosition( GRIPPER_RIGHT_WIDE_OPEN);
        gripperLeft.setPosition( GRIPPER_LEFT_WIDE_OPEN);
    }

    public void gripperClosed(){
        gripperRight.setPosition(GRIPPER_RIGHT_CLOSED);
        gripperLeft.setPosition(GRIPPER_LEFT_CLOSED);
    }
    public void gripperOpen(){
        gripperRight.setPosition(GRIPPER_RIGHT_OPEN);
        gripperLeft.setPosition(GRIPPER_LEFT_OPEN);
    }

    public void gripperRightOpen(){
        gripperRight.setPosition(GRIPPER_RIGHT_OPEN);
    }
    public void gripperLeftOpen(){gripperLeft.setPosition(GRIPPER_LEFT_OPEN);}

    public double getWinchPos(){
        double slidePos;
        slidePos = extendMotor.getCurrentPosition()/ TICKS_PER_SLIDE_IN; //returns in inches
        return  slidePos;
    }
    public void  setSlideLevel_0(){
        targetHeight = ( SLIDE_LEVEL_0 );
        liftToTargetHeight(targetHeight,3);

    }

    public void  setSlideRow_1(){
        targetHeight = (SLIDE_LEVEL_ROW_1 );
        liftToTargetHeight(targetHeight,3);
    }

    public void  setSlideRow_2(){
        targetHeight = (  SLIDE_LEVEL_ROW_2 );
        liftToTargetHeight(targetHeight,3);
    }

    public void  setSlideRow_4(){
        targetHeight = ( SLIDE_LEVEL_ROW_4 );
        liftToTargetHeight(targetHeight,3);
    }

    public void  setSlideRow_6(){
        targetHeight = ( SLIDE_LEVEL_ROW_6 );
        liftToTargetHeight(targetHeight,3);
    }

    public void  setSlidReach_1(){
        targetHeight = (  SLIDE_REACH_1 );
        liftToTargetHeight(targetHeight,3);
    }

    public void  setSlidReach_2(){
        targetHeight = (  SLIDE_REACH_2 );
        liftToTargetHeight(targetHeight,3);
    }
    public void  setSlidReach_3(){
        targetHeight = ( SLIDE_REACH_3);
        liftToTargetHeight(targetHeight,3);
    }

    public void setTurnerLoad(){
        targetAngle = ( TURNER_LOAD_ANGLE );
        rotateToTargetAngle( targetAngle,3, TURNER_SPEED);
    }

    public void setTurnerDeploy(){
        targetAngle = ( TURNER_DEPLOY_ANGLE );
        rotateToTargetAngle( targetAngle,3, TURNER_SPEED);
    }


    /// Get white pixel methods

    public void getPixel_4(){
        angler.setPosition(0.55); // this will change slightly as the arm extends.
        armWheel.setPosition(ARM_WHEEL_PIXEL_4);// use servo and wheel to fine tune height of arm
        rotateToPreciseAngle(PIXEL_4_ANGLE,2);
        liftToTargetHeight(SLIDE_REACH_2,2);
    }

    public void getPixel_5(){
        angler.setPosition(0.55); // this will change slightly as the arm extends.
        armWheel.setPosition(ARM_WHEEL_PIXEL_5);// use servo and wheel to fine tune height of arm
        rotateToPreciseAngle(PIXEL_5_ANGLE,2);
        liftToTargetHeight(SLIDE_REACH_2,2);
    }


    public void slideMechanicalReset(){
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to switch off encoder to run with a timer
        extendMotor.setPower(SLIDERESETSPEED);

        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 2.0) {
            //Time wasting loop so slide can retract. Loop ends when time expires or touch sensor is pressed
        }
        extendMotor.setPower(0);
        // Don't use the "delay loop" when the lift is belt driven (no spring to relax).
        //runtime.reset();
        //while ((runtime.seconds() < 0.25)) {
            //Time wasting loop to let spring relax
        //}
        // set everything back the way is was before reset so encoders can be used
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftToTargetHeight(double height, double timeoutS){
        int newTargetHeight;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_SLIDE_IN);
            // Set the target now that is has been calculated
            extendMotor.setTargetPosition(newTargetHeight);
            // Turn On RUN_TO_POSITION
           extendMotor.setPower(Math.abs(SLIDESPEEDSLOWER));
            // reset the timeout time and start motion.
            runtime.reset();
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (opmode.opModeIsActive() &&
            //       (runtime.seconds() < timeoutS) && slidemotorback.isBusy() && slidemotorfront.isBusy()) {
            // holds up execution to let the slide go up to the right place
            // }
        }
    }

    public void rotateToTargetAngle(double degree, double timeoutS, double TURNER_SPEED){
        int newTargetAngle;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // This is the rotational euivalent of a lift to height
            // The "0" point for the arm on this robot is 25 degrees below horizontal
            // deploy is 120 degrees from horizontal or 145 degrees from start position.

            newTargetAngle = (int)(degree *  TICKS_PER_TURNER_DEGREE);
            // Set the target now that is has been calculated
            armWheel.setPosition(ARM_WHEEL_STOW);// use servo and wheel to fine tune height of arm
            turnerMotor.setTargetPosition(newTargetAngle);
            // Turn On RUN_TO_POSITION
            turnerMotor.setPower(Math.abs(TURNER_SPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            turnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             while (opmode.opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&  turnerMotor.isBusy() ) {
            // holds up execution to let the arm turner do its thing.
            }
        }
    }
    public void rotateToPreciseAngle(double degree, double timeoutS){
        int newTargetAngle;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // This is the rotational euivalent of a lift to height
            // The "0" point for the arm on this robot is 25 degrees below horizontal
            // deploy is 120 degrees from horizontal or 145 degrees from start position.

            newTargetAngle = (int)(degree *  TICKS_PER_TURNER_DEGREE);
            // Set the target now that is has been calculated

            turnerMotor.setTargetPosition(newTargetAngle);
            // Turn On RUN_TO_POSITION
            turnerMotor.setPower(Math.abs(TURNER_PRECISE_SPEED)); // this is a LOW power not enough to make the aem go far
            // reset the timeout time and start motion.
            runtime.reset();
            turnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&  turnerMotor.isBusy() ) {
                // holds up execution to let the arm turner do its thing.
            }
        }
    }
}
