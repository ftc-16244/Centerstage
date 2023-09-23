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
public class LiftGripper {

    //Define Hardware Objects
    public Servo            angler             = null;
    public Servo            gripper        = null; //configis the same name
    public VoltageSensor    voltSensor         = null;


    //Define Hardware Objects

    //Constants for rollers
    public static final double      GRIPPER_INITIAL    = 0.5; // not gripped
    public static final double      GRIPPER_OPEN       = 0.25; // not gripped
    public static final double      GRIPPER_CLOSED      = 0.64 ; // cone gripped
    public static final long    GRIPPER_DELAY            = 300 ; // delay between top and side gripper action (miliseconds)


    //Constants for angler
    public static final double      collect       = 0.17; // facing to the back
    public static final double      deposit      = 0.86; // facing to the front
    public static final double      other      = 0.86; // facing to the front


    public  DcMotorEx       liftMotor;  // config name is "slideMotor"

    Telemetry       telemetry;
    LinearOpMode    opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public  static double           LIFTSPEED                  = 0.70; //
    public  static double           LIFTSPEEDSLOWER            = 0.5; //
    public static  double           LIFTRESETSPEED                 = -0.2; //
    public static final double      LIFT_LEVEL_1                   = 0; // inches Load cone level
    public static final double      LIFT_LEVEL_2                   = 2; // inches ground junction
    public static final double      LIFT_LEVEL_3                   = 13.5; // inches 12" Junction
    public static final double      LIFT_LEVEL_4                   = 23; // inches 24" Junction

    private static final double     LIFT_HEIGHT_CORRECTION_FACTOR   =   1.13;
    private static final double     TICKS_PER_MOTOR_REV             = 145.1; // goBilda 1150  //312 RPM  537.7
    private static final double     PULLEY_DIA                      = 40; // milimeters
    private static final double     LIFT_DISTANCE_PER_REV     = PULLEY_DIA * Math.PI / (25.4*LIFT_HEIGHT_CORRECTION_FACTOR);
    private static final double     TICKS_PER_LIFT_IN               = TICKS_PER_MOTOR_REV / LIFT_DISTANCE_PER_REV;

    public static double            LIFT_NEW_P                     = 10.0; // 2.5 default
    public static double            LIFT_NEW_I                     = 0.5;// 0.1 default
    public static double            LIFT_NEW_D                     = 0.0; // 0.2 default
    public static double            LIFT_NEW_F                     = 0; // 10 default


    public double  targetHeight;


    /// constructor with opmmode passed in
    public LiftGripper(LinearOpMode opmode) {
        this.opmode = opmode;

    }

    public void init(HardwareMap hwMap)  {

        voltSensor = hwMap.voltageSensor.get("Expansion Hub 2");

        // Initialize tuner the servo that rotates the cone capture bucket
        angler = hwMap.get(Servo.class,"angler");// port 4
        //turner.setPosition(BACK);

        // Initialize the left gripper
        gripper = hwMap.get(Servo.class,"gripperLeft"); //port 0


        // Initialize the slide motor
        liftMotor = hwMap.get(DcMotorEx.class,"slideMotor");
        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);

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
    public void anglerCollect() {
        angler.setPosition(collect);//fwd
    }
    public void anglerDeposit() {
        angler.setPosition(deposit); // back
    }
    public void anglerOther() {
        angler.setPosition(other);//fwd
    }



    ///////////////////////////////////////////////////////////////////////////////////////////////

    public void gripperInitTeleop(){
        gripper.setPosition(GRIPPER_INITIAL);
    }

    public void gripperInitAuto(){
        gripper.setPosition(GRIPPER_INITIAL);
    }
    public void gripperClosed(){
        gripper.setPosition(GRIPPER_CLOSED);

    }

    public void gripperOpen(){
        gripper.setPosition(GRIPPER_OPEN);
    }
    // simple getter and setter methods for use in state machines
    public double getSlidePos(){
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

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to swich off encoder to run with a timer
        liftMotor.setPower(LIFTRESETSPEED);
        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        // reset for time allowed or until the limit/ touch sensor is pressed.
        while (runtime.seconds() < 2.0) {

            //Time wasting loop so slide can retract. Loop ends when time expires or tiuch sensor is pressed
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
