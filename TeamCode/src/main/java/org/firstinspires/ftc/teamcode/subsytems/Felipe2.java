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

@Config // this is so the dashboard will pick up variables
public class Felipe2 {

    //Define Hardware Objects
    public Servo            angler             = null;
    public Servo            gripperRight       = null;
    public Servo            gripperLeft        = null;
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

    public static final double      SLIDEWHEEL_POS_1        = 1; //to open more, increase
    public static final double      SLIDEWHEEL_POS_2        = 1; //to open more, increase
    public static final double      SLIDEWHEEL_POS_3        = 1; //to open more, increase


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
    public static  double           SLIDERESETSPEED                 = -0.2; //
    public static final double      SLIDE_LEVEL_1                   = 0; // Load pixel level
    public static final double      SLIDE_LEVEL_1point5             = 2.5; // auto drop pixel in right spot
    public static final double      SLIDE_LEVEL_1point5_white             = 3.8; // auto drop pixel in right spot
    public static final double      SLIDE_LEVEL_1point5_white_RED             = 4.15;
    public static final double      SLIDE_LEVEL_1point5_back             = 4.75; // auto drop pixel in right spot

    public static final double      SLIDE_LEVEL_1point5_back_RED             = 5.25; // auto drop pixel in right spot

    public static final double      SLIDE_LEVEL_2                   = 10;
    public static final double      SLIDE_LEVEL_3                   = 13;
    public static final double      SLIDE_LEVEL_4                   = 19.75;

    private static final double    SLIDE_HEIGHT_CORRECTION_FACTOR   =   1.00;
    private static final double     TICKS_PER_MOTOR_REV             = 290.2; // goBilda 435  //312 RPM  537.7
    private static final double     TICKS_PER_MOTOR_REV_TURNER       = 2850.2; // (1425.1 * 2) goBilda 435  //312 RPM  537.7
    private static final double     PULLEY_DIA                      = 38.2; // milimeters
    private static final double     SLIDE_DISTANCE_PER_REV           = PULLEY_DIA * Math.PI / (25.4 * SLIDE_HEIGHT_CORRECTION_FACTOR);
    private static final double     TICKS_PER_SLIDE_IN               = TICKS_PER_MOTOR_REV / SLIDE_DISTANCE_PER_REV;
    private static final double     TICKS_PER_TURNER_IN               = TICKS_PER_MOTOR_REV_TURNER / 360;


    public double  targetHeight;

    //Constants for Turner
    public static final double      TURNER_DEPLOY      = 145; // deposit the pixel
    public static final double      TURNER_LOAD      = 0; // Loading the pixel


    /// constructor with opmode passed in
    public Felipe2(LinearOpMode opmode) {
        this.opmode = opmode;

    }

    public void init(HardwareMap hwMap)  {

        //voltSensor = hwMap.voltageSensor.get("Expansion Hub 2");

        // Initialize angler
        angler = hwMap.get(Servo.class,"anglerServo"); // port 2
        //setanglerCarry();

        // Initialize the gripper
        gripperRight = hwMap.get(Servo.class,"gripperRightServo"); //port 1
        gripperLeft = hwMap.get(Servo.class,"gripperLeftServo"); //port 2

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
    public void  setSlideLevel1(){
        targetHeight = ( SLIDE_LEVEL_1 );
        liftToTargetHeight(targetHeight,3, SLIDESPEEDSLOWER);
    }

    public void  setSlideLevel1point5(){
        targetHeight = (  SLIDE_LEVEL_1point5 );
        liftToTargetHeight(targetHeight,3, SLIDESPEEDSLOWER);
    }

    public void  setSlideLevel1point5_white(){
        targetHeight = (  SLIDE_LEVEL_1point5_white );
        liftToTargetHeight(targetHeight,3, SLIDESPEEDSLOWER);
    }

    public void  setSlideLevel1point5_white_RED(){
        targetHeight = (  SLIDE_LEVEL_1point5_white_RED );
        liftToTargetHeight(targetHeight,3, SLIDESPEEDSLOWER);
    }

    public void  setSlideLevel1point5_back_RED(){
        targetHeight = (  SLIDE_LEVEL_1point5_back_RED );
        liftToTargetHeight(targetHeight,3, SLIDESPEEDSLOWER);
    }

    public void  setSlideLevel1point5_back(){
        targetHeight = (  SLIDE_LEVEL_1point5_back );
        liftToTargetHeight(targetHeight,3, SLIDESPEEDSLOWER);
    }
    public void setSlideLevel2(){
        targetHeight = ( SLIDE_LEVEL_2);
        liftToTargetHeight(targetHeight,3,  SLIDESPEEDSLOWER);
    }
    public void setSlideLevel3(){
        targetHeight = ( SLIDE_LEVEL_3);
        liftToTargetHeight(targetHeight,3, SLIDESPEED);
    }
    public void setSlideLevel4(){
        targetHeight = ( SLIDE_LEVEL_4);
        liftToTargetHeight(targetHeight,3, SLIDESPEED);
    }

    public void setTurnerLoad(){
        targetHeight = ( TURNER_LOAD );
        rotateToTargetAngle(targetHeight,3, SLIDESPEED);
    }

    public void setTurnerDeploy(){
        targetHeight = ( TURNER_DEPLOY );
        liftToTargetHeight(targetHeight,3, SLIDESPEED);
    }
    public void slideMechanicalReset(){
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to switch off encoder to run with a timer
        extendMotor.setPower(SLIDERESETSPEED);

        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 2.5) {
            //Time wasting loop so slide can retract. Loop ends when time expires or touch sensor is pressed
        }
        extendMotor.setPower(0);
        runtime.reset();
        while ((runtime.seconds() < 0.25)) {
            //Time wasting loop to let spring relax
        }
        // set everything back the way is was before reset so encoders can be used
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftToTargetHeight(double height, double timeoutS, double SLIDELIFTSPEED){
        int newTargetHeight;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_SLIDE_IN);
            // Set the target now that is has been calculated
            extendMotor.setTargetPosition(newTargetHeight);
            // Turn On RUN_TO_POSITION
           extendMotor.setPower(Math.abs(SLIDELIFTSPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (opmode.opModeIsActive() &&
            //       (runtime.seconds() < timeoutS) && slidemotorback.isBusy() && slidemotorfront.isBusy()) {
            // holds up execution to let the slide go up to the right place
            // }
        }
    }

    public void rotateToTargetAngle(double degree, double timeoutS, double SLIDELIFTSPEED){
        int newTargetAngle;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetAngle = (int)(degree *  TICKS_PER_TURNER_IN);
            // Set the target now that is has been calculated
            turnerMotor.setTargetPosition(newTargetAngle);
            // Turn On RUN_TO_POSITION
            turnerMotor.setPower(Math.abs(SLIDELIFTSPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            turnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (opmode.opModeIsActive() &&
            //       (runtime.seconds() < timeoutS) && slidemotorback.isBusy() && slidemotorfront.isBusy()) {
            // holds up execution to let the slide go up to the right place
            // }
        }
    }
}
