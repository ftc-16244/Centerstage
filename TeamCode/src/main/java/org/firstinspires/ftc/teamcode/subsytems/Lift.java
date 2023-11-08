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

@Config // this is so the dashboard will pick up variables
public class Lift {

    //Define Hardware Objects
    public Servo            angler             = null;
    public Servo            gripper            = null;
    public VoltageSensor    voltSensor         = null;

    public  DcMotorEx       liftMotor;  // config name is "slideMotor"

    //Constants for gripper

    public static final double      GRIPPER_OPEN       = 0.30; // not gripped
    public static final double      GRIPPER_CLOSED      = 0.46; // pixel gripped

    //Constants for angler
    //NOTE: lower values make the angler go higher, higher values make it go lower
    public static final double      ANGLER_CARRY       = 0.442 ; // load and moving the pixel
    public static final double      ANGLER_DEPLOY      = 0.46; // deposit the pixel
    public static final double      ANGLER_LOAD      = 0.496; // Loading the pixel

    Telemetry       telemetry;
    LinearOpMode    opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public  static double           LIFTSPEED                  = 1.00; // full speed
    public  static double           LIFTSPEEDSLOWER            = 0.5; //half speed
    public static  double           LIFTRESETSPEED                 = -0.2; //
    public static final double      LIFT_LEVEL_1                   = 0; // Load pixel level
    public static final double      LIFT_LEVEL_1point5             = 1; // auto drop pixel in right spot
    public static final double      LIFT_LEVEL_2                   = 4;
    public static final double      LIFT_LEVEL_3                   = 9;
    public static final double      LIFT_LEVEL_4                   = 13;

    private static final double     LIFT_HEIGHT_CORRECTION_FACTOR   =   1.00;
    private static final double     TICKS_PER_MOTOR_REV             = 384.5; // goBilda 435  //312 RPM  537.7
    private static final double     PULLEY_DIA                      = 40; // milimeters
    private static final double     LIFT_DISTANCE_PER_REV           = PULLEY_DIA * Math.PI / (25.4 * LIFT_HEIGHT_CORRECTION_FACTOR);
    private static final double     TICKS_PER_LIFT_IN               = TICKS_PER_MOTOR_REV / LIFT_DISTANCE_PER_REV;

    public double  targetHeight;

    /// constructor with opmode passed in
    public Lift(LinearOpMode opmode) {
        this.opmode = opmode;

    }

    public void init(HardwareMap hwMap)  {

        voltSensor = hwMap.voltageSensor.get("Expansion Hub 4");

        // Initialize angler
        angler = hwMap.get(Servo.class,"anglerServo"); // port 2
        //setanglerCarry();

        // Initialize the gripper
        gripper = hwMap.get(Servo.class,"gripperServo"); //port 0

        // Initialize the lift motor
        liftMotor = hwMap.get(DcMotorEx.class,"liftMotor");
        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidfOrig = liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        //PIDFCoefficients pidSlide_New = new PIDFCoefficients(SLIDE_NEW_P, SLIDE_NEW_I, SLIDE_NEW_D, SLIDE_NEW_F);
        //slidemotorback.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);
        //slidemotorfront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);
        // re-read coefficients and verify change.
        //PIDFCoefficients pidModifiedback = slidemotorback.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //PIDFCoefficients pidModifiedfront = slidemotorfront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //slidemotorback.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);
        //slidemotorfront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSlide_New);

        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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

    public void gripperClosed(){
        gripper.setPosition(GRIPPER_CLOSED);
    }
    public void gripperOpen(){
        gripper.setPosition(GRIPPER_OPEN);
    }
    public double getWinchPos(){
        double liftPos;
        liftPos = liftMotor.getCurrentPosition()/ TICKS_PER_LIFT_IN; //returns in inches
        return  liftPos;
    }
    public void  setSlideLevel1(){
        targetHeight = ( LIFT_LEVEL_1 );
        liftToTargetHeight(targetHeight,3, LIFTSPEEDSLOWER);
    }
    public void setSlideLevel2(){
        targetHeight = ( LIFT_LEVEL_2);
        liftToTargetHeight(targetHeight,3,  LIFTSPEEDSLOWER);
    }
    public void setSlideLevel3(){
        targetHeight = ( LIFT_LEVEL_3);
        liftToTargetHeight(targetHeight,3, LIFTSPEED);
    }
    public void setSlideLevel4(){
        targetHeight = ( LIFT_LEVEL_4);
        liftToTargetHeight(targetHeight,3, LIFTSPEED);
    }
    public void slideMechanicalReset(){
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to switch off encoder to run with a timer
        liftMotor.setPower(LIFTRESETSPEED);

        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 2.0) {
            //Time wasting loop so slide can retract. Loop ends when time expires or touch sensor is pressed
        }
        liftMotor.setPower(0);
        runtime.reset();
        while ((runtime.seconds() < 0.25)) {
            //Time wasting loop to let spring relax
        }
        // set everything back the way is was before reset so encoders can be used
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slideTrainerState = SlideTrainerState.IDLE;// once this is done we are at zero power or idling.
    }

    public void liftToTargetHeight(double height, double timeoutS, double SLIDELIFTSPEED){
        int newTargetHeight;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_LIFT_IN);
            // Set the target now that is has been calculated
            liftMotor.setTargetPosition(newTargetHeight);
            // Turn On RUN_TO_POSITION
            liftMotor.setPower(Math.abs(SLIDELIFTSPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (opmode.opModeIsActive() &&
            //       (runtime.seconds() < timeoutS) && slidemotorback.isBusy() && slidemotorfront.isBusy()) {
            // holds up execution to let the slide go up to the right place
            // }
        }
    }
}
