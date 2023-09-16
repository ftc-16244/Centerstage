package org.firstinspires.ftc.teamcode.Teleop2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class Quest_Test_File extends LinearOpMode {


   // Declare OpMode members.
   private ElapsedTime  runtime = new ElapsedTime();
   private DcMotor      backBack = null;
   private DcMotor      liftMotor = null; //slidemotor
   private DcMotor      leftBack = null;
   private DcMotor      rightBack = null;
   public  DcMotor      linearMotor = null;

   private Servo clawServo = null;

   // Claw Position Settings

   private double clawOpen = 0.52; // this is a 5 turn servo so the numbers are close together
   private double clawClosed = 0.60;

   //lift calculations to get from motor ticks to inches of lift
   //the code should use inches as the human input and convert to ticks for the motor controller
   private static final double LIFT_HEIGHT_CORRECTION_FACTOR = 1.13;
   private static final double TICKS_PER_MOTOR_REV = 384.5; // goBilda 1150  //312 RPM  537.7 // 435 RM 384.5
   private static final double PULLEY_DIA = 40; // millimeters
   private static final double SLIDE_LIFT_DISTANCE_PER_REV = PULLEY_DIA * Math.PI / (25.4 * LIFT_HEIGHT_CORRECTION_FACTOR);
   private static final double TICKS_PER_LIFT_IN = TICKS_PER_MOTOR_REV / SLIDE_LIFT_DISTANCE_PER_REV;
   public static final double SLIDELIFTSPEED = 1;

   // Lift Levels change heights as needed. Once set they should not change much.
   // Low batteries or string problems can trick you into wanting to change these

   public static final double      SLIDE_LEVEL_1                   = 0; // inches Load cone level GROUND
   public static final double      SLIDE_LEVEL_2                   = 2; // inches ground junction
   public static final double      SLIDE_LEVEL_3                   = 13.5; // inches 12" Junction
   public static final double      SLIDE_LEVEL_4                   = 23; // inches 24" Junction

   // This variable is a class variable becase it gets shared with several of the methods below.
   double targetHeight;





   @Override
   public void runOpMode() {
      telemetry.addData("Status", "Initialized");
      telemetry.update();


      // Initialize the hardware variables. Note that the strings used here as parameters
      // to 'get' must correspond to the names assigned during the robot configuration
      // step (using the FTC Robot Controller app on the phone).
      backBack = hardwareMap.get(DcMotor.class, "back_back");
      leftBack = hardwareMap.get(DcMotor.class, "left_back");
      rightBack = hardwareMap.get(DcMotor.class, "right_back");
      linearMotor = hardwareMap.get(DcMotor.class, "lift");
      clawServo = hardwareMap.get(Servo.class, "claw_servo");


      // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
      // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
      // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
      backBack.setDirection(DcMotor.Direction.REVERSE);//
      leftBack.setDirection(DcMotor.Direction.REVERSE);
      rightBack.setDirection(DcMotor.Direction.FORWARD);

      linearMotor.setDirection(DcMotor.Direction.FORWARD);//
      linearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


      // Wait for the game to start (driver presses PLAY)
      waitForStart();
      runtime.reset();

      // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {

         // Setup a variable for each drive wheel to save power level for telemetry
         double leftPower;
         double rightPower;






         // POV Mode uses left stick to go forward, and right stick to turn.
         // - This uses basic math to combine motions and is easier to drive straight.
         double drive = -gamepad1.left_stick_y;
         double turn = gamepad1.right_stick_x;
         leftPower = Range.clip(drive + turn, -1.0, 1.0);
         rightPower = Range.clip(drive - turn, -1.0, 1.0);


         // Send calculated power to wheels
         leftBack.setPower(leftPower);
         rightBack.setPower(rightPower);
      }
      //linearMotor.setPower(liftPower);

      if (gamepad1.left_trigger > 0.25) {
         clawServo.setPosition(clawClosed);
      }
      if (gamepad1.right_trigger > 0.25) {
         clawServo.setPosition(clawOpen);

      // Assign gamepad buttons to a methods for moving the slide.
      }
      if (gamepad2.dpad_left) {
         setSlideLevel2();
      }

       if (gamepad2.dpad_up) {
       setSlideLevel3();
      }

       if (gamepad2.dpad_down) {
       setSlideLevel1();

       }


   }
   // Simple methods called up by gamepad button to pass info the main lift method.
   // You can add some other commands here to do multiple functions when the button is pressed.
   //an example is opening the claw each time you lower the lift to the ground.

   // make the timeout as short as possible. If it is too short the lift will stop moving
   // before it reaches the fill height
   // i you make it too long if makes the robot harder to drive because the lift loop can interfwwer
   // with the joystick functions when the code is help up in the loop to make the loft go up.

   public void setSlideLevel3() {

      targetHeight = (SLIDE_LEVEL_3);
      liftToTargetHeight(targetHeight, 3, SLIDELIFTSPEED);

   }

   public void setSlideLevel2() {

      targetHeight = (SLIDE_LEVEL_1);
      liftToTargetHeight(targetHeight, 3, SLIDELIFTSPEED);

   }

   public void setSlideLevel1() {

      targetHeight = (SLIDE_LEVEL_1);
      liftToTargetHeight(targetHeight, 3, SLIDELIFTSPEED);

   }

   // This is the main method that handles the lift. It keeps the code in a while loop until the loft
   // reaches its target. The timeout is to allow the code to escape the loop in case it can't get to the
   // target due to a low battery, mechanical problem etc. You have to get out of the loop to do other
   // things with the robot. There can be a little interference with driving at times.


   public void liftToTargetHeight(double height, double timeoutS, double SLIDELIFTSPEED) {

      int newTargetHeight;


      // Ensure that the opmode is still active
      if (opModeIsActive()) {

         // Determine new target lift height in ticks based on the current position.
         // When the match starts the current position should be reset to zero.

         newTargetHeight = (int) (height * TICKS_PER_LIFT_IN);
         // Set the target now that is has been calculated - TARGET HEIGHT IN TICKS
         linearMotor.setTargetPosition(newTargetHeight);

         // Turn On RUN_TO_POSITION - SPEED
         linearMotor.setPower(Math.abs(SLIDELIFTSPEED));

         // reset the timeout time and start motion.
         runtime.reset();
         linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         while (opModeIsActive() &&
               (runtime.seconds() < timeoutS) && linearMotor.isBusy() ) {


          }

         // Show the elapsed game time, drive wheel power and claw position.
         telemetry.addData("Status", "Run Time: " + runtime.toString());
         telemetry.addData("Motors", "Left (%.2f), Right (%.2f)", leftBack, rightBack);
         telemetry.addData("Servo Position", clawServo.getPosition());

         telemetry.update();

      }
   }
}
