package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Arrays;

import static java.lang.Thread.sleep;

@Config // this is so the dashboard will pick up variables
public class Felipe2 {

    //Define Hardware Objects
    private Servo            angler             = null;
    private Servo            gripperRight       = null;
    private Servo            gripperLeft        = null;
    private Servo            armWheel            = null;
    private DcMotorEx       extendMotor;  // config name is "slideMotor"
    private DcMotorEx        turnerMotor; // config name is "turnerMotor"

    //Constants for gripper
    //larer numbers are more clockwise

    private static final double      GRIPPER_LEFT_CLOSED      = 0.44; //to close more, decrease
    private static final double      GRIPPER_LEFT_OPEN        = 0.69; //to open more, increase
    private static final double      GRIPPER_LEFT_WIDE_OPEN   = 0.8;

    private static final double      GRIPPER_RIGHT_CLOSED     = 0.52;// to close more, increase
    private static final double      GRIPPER_RIGHT_OPEN       = 0.31; // too open more, decrease
    private static final double      GRIPPER_RIGHT_WIDE_OPEN  = 0.15; // not gripped

    //Constants for slidewheel

    private static final double      ARM_WHEEL_PIXEL_5        = 1; //stowed position
    private static final double      ARM_WHEEL_PIXEL_4        = 1; //to open more, increase
    private static final double      ARM_WHEEL_STOW        = 0.62; //pick up 4th and 5th pixel on stack
    private static final double      ARM_WHEEL_PIXEL_1      = 0.58; //pick up ground pixel


    //Constants for angler
    //NOTE: lower values make the angler go higher, higher values make it go lower
    private static final double      ANGLER_CARRY       = 0.75;// load and moving the pixel
    private static final double      ANGLER_DEPLOY      = 0.1; // deposit the pixel
    private static final double      ANGLER_LOAD      = 0.7; // Loading the pixel
    public static final double      ANGLER_AUTO      = 0.65; // Loading the pixel

    Telemetry       telemetry;
    LinearOpMode    opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    private static final double           SLIDESPEED                  = 1.0; // full speed
    private static final double           SLIDESPEEDSLOWER            = 0.5; //half speed
    private static final double           SLIDERESETSPEED             = -1.0; // only used to retract and reset slide encoder
    private static final double      SLIDE_LEVEL_0               = 0;// Extension fully retracted but not to mechanical stop
    private static final double      SLIDE_LEVEL_ROW_1           = 4.0 ; // First yellow auto high accuracy -measured 2/7
    private static final double      SLIDE_LEVEL_ROW_2           = 8;  // 8 Second row of pixels
    private static final double      SLIDE_LEVEL_ROW_4           = 16; // 15 check wire mgt before making 16
    private static final double      SLIDE_LEVEL_ROW_6           = 21.5; // 20 check wire mgt before making 24

    private static final double      SLIDE_REACH_0         = 0.25; // Used to reach out horizontally to drop purple ot get white
    private static final double      SLIDE_REACH_1         = 0.0; // Used to reach out horizontally to drop purple ot get white
    private static final double      SLIDE_REACH_2         = 0.0; // 8 Yellow pixel gentle drop (first yellow)
    private static final double      SLIDE_REACH_3         = 0.0; // wire check
    private static final double      SLIDE_REACH_4         = 0.0; //wire check

    private static final double     SLIDE_HEIGHT_CORRECTION_FACTOR   =   1.00;
    private static final double     TICKS_PER_MOTOR_REV_SLIDE             = 145.1 * 2; // goBilda 1150 RPM MOTOR and 2:1 Bevel Gear

    /////////////
    // TURNER ARM ROTATOR

    private static final double     TICKS_PER_MOTOR_REV_TURNER       = 2850.2; // (1425.1 * 2) goBilda 435  //312 RPM  537.7
    private static final double     PULLEY_DIA                      = 38.2; // milimeters
    private static final double     SLIDE_DISTANCE_PER_REV           = PULLEY_DIA * Math.PI / (25.4 * SLIDE_HEIGHT_CORRECTION_FACTOR);
    private static final double     TICKS_PER_SLIDE_IN               = TICKS_PER_MOTOR_REV_SLIDE / SLIDE_DISTANCE_PER_REV;
    private static final double     TICKS_PER_TURNER_DEGREE             = TICKS_PER_MOTOR_REV_TURNER / 360;

    private static final double     TURNER_SPEED = 1.0;
    private static final double     TURNER_PRECISE_SPEED = 0.35; // was 0.2 used to help the arm float with arm wheel
    //Constants for Turner
    private static final double      TURNER_DEPLOY_ANGLE =  145.0; // deposit the pixel
    private static final double      TURNER_DRONE_ANGLE = 70.0;
    private static final double      TURNER_LOAD_ANGLE      = 0.0; // Loading the pixel
    private static final double      PIXEL_4_ANGLE =10; // pick up 4th and possibly 5th pixel from the mat.
    private static final double      PIXEL_5_ANGLE =11; // pick up top or 5th pixel only from white stack
    public double  targetHeight;
    public double  targetAngle;
    public boolean turnerDown = true;

    /// constructor with opmode passed in
    public Felipe2(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void init(HardwareMap hwMap)  {

        // Initialize angler
        angler = hwMap.get(Servo.class,"angler"); // Exp Hub port 0
        //setanglerCarry();

        // Initialize the gripper
        gripperRight = hwMap.get(Servo.class,"gripperRight"); //Exp Hub port 4
        gripperLeft = hwMap.get(Servo.class,"gripperLeft"); //Exp Hub port 2
        armWheel = hwMap.get(Servo.class,"armWheel"); //Exp Hub port 3

        // Initialize the lift motor
        extendMotor = hwMap.get(DcMotorEx.class,"lift");
        turnerMotor = hwMap.get(DcMotorEx.class, "turner");
        extendMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turnerMotor.setDirection(DcMotorEx.Direction.FORWARD); // was forward

        turnerMotor.setCurrentAlert(5.0, CurrentUnit.AMPS);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidSlide_New = new PIDFCoefficients(14, 0, 1.5, 0); // p was 10
        extendMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);

        PIDFCoefficients pidTurner_New = new PIDFCoefficients(12, 0, 3.5, 0);
        turnerMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidTurner_New);

        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        turnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turnerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        armWheel.setPosition(ARM_WHEEL_STOW);

        //slideMechanicalReset();
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
    public void setAnglerAuto() {
        angler.setPosition(ANGLER_AUTO);//fwd
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

    public void  setSlideReach_1(){
        targetHeight = (  SLIDE_REACH_1 );
        liftToTargetHeight(targetHeight,3);
    }

    public void  setSlideReach_2(){
        targetHeight = (  SLIDE_REACH_2 );
        liftToTargetHeight(targetHeight,3);
    }
    public void  setSlideReach_3(){
        targetHeight = ( SLIDE_REACH_3);
        liftToTargetHeight(targetHeight,3);
    }

    //PASS IN A DOUBLE NO INTEGERS has to have a decimal
    public void setTurnerLoad(){
        setAnglerDeploy();
        setSlideLevel_0();
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        targetAngle = ( TURNER_LOAD_ANGLE );
        rotateToTargetAngle( targetAngle,1, TURNER_SPEED);
        turnerDown = true;
    }

    public void setTurnerDeploy(){
        targetAngle = ( TURNER_DEPLOY_ANGLE );
        rotateToTargetAngle( targetAngle,1, TURNER_SPEED);
        if (extendMotor.getTargetPosition() < 50) {
            //slideMechanicalReset();
        }
        turnerDown = false;
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
    public void startSlideMechanicalReset() {
        rotateToTargetAngle(25, 0.5, 30);

        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to switch off encoder to run with a timer
        extendMotor.setPower(SLIDERESETSPEED);

        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 1.0) {
            //Time wasting loop so slide can retract. Loop ends when time expires
        }

        // set everything back the way is was before reset so encoders can be used
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setPower(0);

        setSlideLevel_0();
        turnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to switch off encoder to run with a timer
        turnerMotor.setPower(-0.3);

        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 1.0) {
            //Time wasting loop so slide can retract. Loop ends when time expires
        }

        // set everything back the way is was before reset so encoders can be used
        turnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turnerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turnerMotor.setPower(0);
    }
    public void slideMechanicalReset(){
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to switch off encoder to run with a timer
        extendMotor.setPower(SLIDERESETSPEED);

        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 1.0) {
            //Time wasting loop so slide can retract. Loop ends when time expires
        }

        // set everything back the way is was before reset so encoders can be used
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setPower(0);

        setSlideLevel_0();

        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 0.75) {
            //Time wasting loop so slide can retract. Loop ends when time expires
        }
    }

    public void liftToTargetHeight(double height, double timeoutS){
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int newTargetHeight;

        // Ensure that the opmode is still active
        if ((opmode.opModeInInit() && !opmode.isStopRequested()) || opmode.opModeIsActive()) {
            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_SLIDE_IN);
            // Set the target now that is has been calculated
            extendMotor.setTargetPosition(newTargetHeight);
            // Turn On RUN_TO_POSITION
           extendMotor.setPower(Math.abs(SLIDESPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /* while ((opmode.opModeInInit() && !opmode.isStopRequested()) || opmode.opModeIsActive() && runtime.seconds() < timeoutS) {
            // holds up execution to let the slide go up to the right place
            }
             */
        }
    }

    public void rotateToTargetAngle(double degree, double timeoutS, double TURNER_SPEED){
        turnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int newTargetAngle;

        // Ensure that the opmode is still active
        if ((opmode.opModeInInit() && !opmode.isStopRequested()) || opmode.opModeIsActive()) {
            // This is the rotational equivalent of a lift to height
            // The "0" point for the arm on this robot is 25 degrees below horizontal
            // deploy is 120 degrees from horizontal or 145 degrees from start position.

            newTargetAngle = (int)(degree *  TICKS_PER_TURNER_DEGREE);
            // Set the target now that is has been calculated//
            //armWheel.setPosition(ARM_WHEEL_STOW);// use servo and wheel to fine tune height of arm
            turnerMotor.setTargetPosition(newTargetAngle);
            // Turn On RUN_TO_POSITION
            turnerMotor.setPower(Math.abs(TURNER_SPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            turnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while ((opmode.opModeInInit() && !opmode.isStopRequested()) || opmode.opModeIsActive() && runtime.seconds() < timeoutS) {
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
            while ((opmode.opModeInInit() && !opmode.isStopRequested()) || opmode.opModeIsActive() && runtime.seconds() < timeoutS) {
                // holds up execution to let the arm turner do its thing.
            }
        }
    }
}
