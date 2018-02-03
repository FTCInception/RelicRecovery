package org.firstinspires.ftc.teamcode.nolan;

/**
 * Created by nplaxton on 12/22/17
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


//@com.qualcomm.robotcore.eventloop.  opmode.TeleOp(name="Nolan v3", group="TeleOp")
@Autonomous(name="Blue_Position_FAR", group="BLUE_FAR")

public class BLUE_Autonomous_FAR_OFFICIAL extends LinearOpMode{

    public static final String TAG = "Vuforia VuMark";
    private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor, relic_motor;
    private static Servo gripperFlipper, relicGripper, relicTooth;
    private static Servo  l_t_gripper, r_t_gripper, l_b_gripper, r_b_gripper;
    private static DcMotor lifter_motor;
    private static Servo jewel_hand, jewel_elbow;
    private final int TICS_PER_REV = 1120;
    private final double INCHES_PER_TIC = 0.01121997376;
    private final double wheelRatio = (25/16);
    private static boolean open_gripper;

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    public void initialize() {
        l_f_motor = hardwareMap.dcMotor.get("left_front");
        l_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_b_motor = hardwareMap.dcMotor.get("left_back");
        l_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        r_f_motor = hardwareMap.dcMotor.get("right_front");
        r_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_b_motor = hardwareMap.dcMotor.get("right_back");
        r_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        //NOTE In the example the DcMotorController.RunMode was used, but this wasn't working so I changed it to DcMotor.RunMode
        relic_motor = hardwareMap.dcMotor.get("relic_motor");
        relic_motor.setDirection(DcMotorSimple.Direction.REVERSE);


   /* Arm: */
        lifter_motor = hardwareMap.dcMotor.get("intake_arm");
        lifter_motor.setDirection(DcMotorSimple.Direction.REVERSE);

   /* GRIPPERS: */
        l_b_gripper = hardwareMap.servo.get("left_bottom_arm");
        r_b_gripper = hardwareMap.servo.get("right_bottom_arm");
        l_t_gripper = hardwareMap.servo.get("left_top_arm");
        r_t_gripper = hardwareMap.servo.get("right_top_arm");
        gripperFlipper = hardwareMap.servo.get("gripper_flipper");

        relicGripper = hardwareMap.servo.get("relic_arm");
        relicTooth = hardwareMap.servo.get("relic_tooth");

   /*Jewel servo and arm*/
        jewel_elbow = hardwareMap.servo.get("la_elbow");
        jewel_hand = hardwareMap.servo.get("la_arm");

        open_gripper = true;

        //not sure about these positions
        jewel_elbow.setPosition(.6);
        jewel_hand.setPosition(0.4);

    }

    public void flipGrip(boolean std)
    {
        if (std)
            gripperFlipper.setPosition(.5);
        else
            gripperFlipper.setPosition(0);
    }

    //This method converts the distance in inches to distance in number of tics
    public int getDistance(double inches)
    {
        return (int)(Math.floor(inches/INCHES_PER_TIC));
    }
    public void stopDriving()
    {
        r_f_motor.setPower(0);
        r_b_motor.setPower(0);
        l_f_motor.setPower(0);
        l_b_motor.setPower(0);
    }
    public void raiseArm(int distance, double power)
    {
        //distance for full flip = 560 (1120 / 2)
        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_motor.setTargetPosition(distance*3);
        lifter_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter_motor.setPower(power);
        while (lifter_motor.isBusy()){

        }
        lifter_motor.setPower(0);
        lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void drive(double power) {

        r_f_motor.setPower(power);
        r_b_motor.setPower(power);
        l_f_motor.setPower(power);
        l_b_motor.setPower(power);

    }
    //Movement handles all driving for the robot during autonomous EXCEPT turning
    public void movement(int distance, double power) {

            r_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            r_f_motor.setTargetPosition((int)(distance/wheelRatio));
            r_b_motor.setTargetPosition((int)(distance/wheelRatio));
            l_f_motor.setTargetPosition((int)(distance/wheelRatio));
            l_b_motor.setTargetPosition((int)(distance/wheelRatio));

            r_f_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r_b_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l_f_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l_b_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive(power);

            while (r_f_motor.isBusy() && r_b_motor.isBusy() && l_f_motor.isBusy() && l_b_motor.isBusy() && opModeIsActive()){
                //Jokes on you kid there's nothing here get good
            }

            stopDriving();
            r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    //This method handles 90 degree turns during autonomous
    public void turn(boolean ccw)
    {
        ElapsedTime eTime = new ElapsedTime();
         if (ccw){
            while (eTime.time()<6 && opModeIsActive()){
                r_f_motor.setPower(-.2);
                r_b_motor.setPower(-.2);
                l_f_motor.setPower(.2);
                l_b_motor.setPower(.2);
            }
        }
        else{
            while (eTime.time()<6 && opModeIsActive()){
                r_f_motor.setPower(.2);
                r_b_motor.setPower(.2);
                l_f_motor.setPower(-.2);
                l_b_motor.setPower(-.2);
            }
        }
        r_f_motor.setPower(0);
        r_b_motor.setPower(0);
        l_f_motor.setPower(0);
        l_b_motor.setPower(0);
        eTime.reset();
    }
    public void strafe(boolean right, double time)
    {
        ElapsedTime eTime = new ElapsedTime();
        if (right){
            while (eTime.time()<time && opModeIsActive()){
                r_f_motor.setPower(-.2);
                r_b_motor.setPower(.2);
                l_f_motor.setPower(.2);
                l_b_motor.setPower(-.2);
            }
        }
        else{
            while (eTime.time()<time && opModeIsActive()){
                r_f_motor.setPower(.2);
                r_b_motor.setPower(-.2);
                l_f_motor.setPower(-.2);
                l_b_motor.setPower(.2);
            }
        }
        r_f_motor.setPower(0);
        r_b_motor.setPower(0);
        l_f_motor.setPower(0);
        l_b_motor.setPower(0);
        eTime.reset();
    }
    /*
    This was just a random guess method but it uses etime which I want to get rid of, I'll replace this later but for debugging I dont want to make things confusing
    public void prayer(ElapsedTime eTime)
    {
        //Guesses the cryptobox rather than trying to read key, effectively what we did at December scrimmage
        sleep(1000);
        jewel_hand.setPosition(.5);
        jewel_elbow.setPosition(.4);
        lifter_motor.setPower(.5); //move lift slightly up, this might be more than slightly though - needs testing - Peter
        sleep(500);
        lifter_motor.setPower(0);
        //start moving forward
        movement(-.25, -0.25, -.25, -.25, eTime, 2);
        //strafe right
        movement(.25, -0.25, -0.25, 0.25, eTime, 1.5);
        movement(0, 0, 0, 0, eTime, .5);
        //open the grippers to release the block
        l_gripper.setPosition(0.2);
        r_gripper.setPosition(0);
        movement(-.15, -.15, -.15, -.15, eTime, 2); //Made this the glyph-shove (official term) - Peter
        //back up to not be touching the glyph
        movement(.15, .15, .15, .15, eTime, 1);
    }
    */

    //This method handles getting off the balancing stone and getting in the right position (touching the balancing stone)
    public void getOff()
    {
        movement(getDistance(28.0),.25);
        turn(true);
        //Strafe into the balancing stone, establishing location
        strafe(false, 4.0);
        movement(getDistance(30.0),.25);

        //we end this method 9 inches from the wall,lined up with the balancing stone, and turned around


    }
    //These methods handle moving to each cryptobox column during autonomous, as determined by the result of the vumark key
    //public void right()
    {
        strafe(true,.4);
        movement(getDistance(5.0),-.25);
        //l_b_gripper.setPower(-.2);
        //r_b_gripper.setPower(-.2);
        sleep(2000);
        //l_b_gripper.setPower(0);
        //r_b_gripper.setPower(0);
        strafe(true,.5);

    }
    public void left()
    {
        strafe(true,4);
        movement(getDistance(5.0),-.25);
        //l_b_gripper.setPower(-.2);
      //  r_b_gripper.setPower(-.2);
        sleep(2000);
     //   l_b_gripper.setPower(0);
     //  r_b_gripper.setPower(0);
        strafe(false,.5);
    }
    public void center()
    {
        strafe(true,2);
        movement(getDistance(5.0),-.25);
      //  l_b_gripper.setPower(-.2);
     //  r_b_gripper.setPower(-.2);
        sleep(2000);
     //   l_b_gripper.setPower(0);
     //   r_b_gripper.setPower(0);
    }

    @Override
    public void runOpMode() {
        initialize();
        //Get Vuforia stuff ready
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AUOhDYf/////AAAAmdDp0b8XRUxwrHihX1jNl15KBIqZaXwn3EnrN6lJDn3BlYAco4a9MdghQnfu+QX+MUaAV3x8eOywibefAugDVdRRLjjYJGSNbljXvztRpkFYIrLZPfKGwHKibcGilsdTGiQebl4+lnGWCY4CzVx7lVFuXY6qWQ5lFAOGaJEUOHvQ1/R8iWUOyOg9RRXalnDsSsvvIoGN3cvqxq4xfcIb2r/Az+UQQTsi73p+GVzrYMDegDQ422tXTcSaLO5Kp6MA6+OfyuuF19nSsTd+5L6Zn3se9oVdsq+fcmffIzoVHaCVT+3rEVqQBngW+viXAvcHMATyVyZB2ZOG3sgBCQDpjPYZ0sKClH56Zj3x9RPnHFeI";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicBoy = relicTrackables.get(0);



        ColorSensor sensorColor;
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");

        waitForStart();

        relicTrackables.activate();
        //start by closing the gripper on the block



        //rotate the Linear Actuator to be parallel to the ground
        jewel_hand.setPosition(1);
        jewel_elbow.setPosition(.3575);



        if (sensorColor.red() < sensorColor.blue()) {

            //the color of the ball facing the sensor is red
            //we need to turn our arm to hit it off
            sleep(500);
            jewel_elbow.setPosition(.1);
            sleep(100);
            jewel_elbow.setPosition(.5);
            sleep(100);
            jewel_elbow.setPosition(.1);
            sleep(100);
            jewel_elbow.setPosition(.5);
            telemetry.addData("Color found is", "red");


        } else if (sensorColor.blue() < sensorColor.red()) {

            //the color of the ball facing the sensor is blue
            //we need to turn our arm to hit it off
            sleep(500);
            jewel_elbow.setPosition(1);
            sleep(100);
            jewel_elbow.setPosition(.5);
            sleep(100);
            jewel_elbow.setPosition(1);
            sleep(100);
            jewel_elbow.setPosition(.5);
            telemetry.addData("Color found is", "blue");


        } else //we find no color ball and we freak out internally - Peter
        {
            telemetry.addData("Color found is", "none lol git gud scrub");
            sleep(2000);
        }
        telemetry.update();
        jewel_elbow.setPosition(.4);
        jewel_hand.setPosition(.5);

        boolean found = false;
        ElapsedTime etime = new ElapsedTime();
        while (!found && etime.time()<15) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicBoy);

            telemetry.addData("VuMark", "%s visible", vuMark);

            if (vuMark == RelicRecoveryVuMark.CENTER) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                getOff();
                center();
                found = true;
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                getOff();
                //right();
                found = true;
            }
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                getOff();
                left();
                found = true;
            }



            if (found == false) {
                telemetry.addData("VuMark", "not visible");
                //prayer(eTime);
            }
            telemetry.update();
        }
        //WE COULDN'T READ THE VUMARK AND SUFFER
        if (!found)
        {
            telemetry.addData("We didn't find it...", "Hold this L");
            getOff();
            center();
        }
        telemetry.update();



    }
    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
