package org.firstinspires.ftc.teamcode.nolan;
/**
 * Created by nplaxton on 12/22/17.
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


    private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static Servo l_b_gripper, r_b_gripper, l_t_gripper, r_t_gripper, gripperFlipper, relicGripper, relicTooth;
    private static DcMotor lifter_motor;
    private static Servo jewel_hand, jewel_elbow;
    private final int TICS_PER_REV = 1120;
    private final double INCHES_PER_TIC = 0.01121997376;
    private static boolean open_gripper;


    public void initialize() {
        l_f_motor = hardwareMap.dcMotor.get("left_front");
        l_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        l_b_motor = hardwareMap.dcMotor.get("left_back");
        l_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_f_motor = hardwareMap.dcMotor.get("right_front");
        r_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        r_b_motor = hardwareMap.dcMotor.get("right_back");
        r_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //NOTE In the example the DcMotorController.RunMode was used, but this wasn't working so I changed it to DcMotor.RunMode



   /* Arm: */
        lifter_motor = hardwareMap.dcMotor.get("intake_arm");
        lifter_motor.setDirection(DcMotorSimple.Direction.REVERSE);

   /* GRIPPERS: */
        l_b_gripper = hardwareMap.servo.get("left_bottom_arm");
        r_b_gripper = hardwareMap.servo.get("right_bottom_arm");
        l_t_gripper = hardwareMap.servo.get("left_top_arm");
        r_t_gripper = hardwareMap.servo.get("right_top_arm");
        gripperFlipper = hardwareMap.servo.get("gripper_flipper");

   /*Jewel servo and arm*/
        jewel_elbow = hardwareMap.servo.get("la_elbow");
        jewel_hand = hardwareMap.servo.get("la_arm");

        open_gripper = true;

        //not sure about these positions
        jewel_elbow.setPosition(0.4);
        jewel_hand.setPosition(0.5);
        l_b_gripper.setPosition(0.4);
        r_b_gripper.setPosition(0.1);
    }

    public void flipGrip(boolean std)
    {
        if (std)
            gripperFlipper.setPosition(.5);
        else
            gripperFlipper.setPosition(0);
    }
    public void releaseBlock(){
        l_b_gripper.setPosition(0.6);
        r_b_gripper.setPosition(0.0);
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
        lifter_motor.setTargetPosition(distance);
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

            r_f_motor.setTargetPosition(distance);
            r_b_motor.setTargetPosition(distance);
            l_f_motor.setTargetPosition(distance);
            l_b_motor.setTargetPosition(distance);

            r_f_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r_b_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l_f_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l_b_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive(power);

            while (r_f_motor.isBusy() && r_b_motor.isBusy() && l_f_motor.isBusy() && l_b_motor.isBusy()){
                //Jokes on you kid there's nothing here get good
            }

            stopDriving();
            r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    //This method handles 90 degree turns during autonomous
    public void turn(boolean cw)
    {
        ElapsedTime eTime = new ElapsedTime();
        if (cw){
            while (eTime.time()<1.5){
                r_f_motor.setPower(-.2);
                r_b_motor.setPower(-.2);
                l_f_motor.setPower(.2);
                l_b_motor.setPower(.2);
            }
        }
        else{
            while (eTime.time()<1.5){
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
            while (eTime.time()<time){
                r_f_motor.setPower(-.2);
                r_b_motor.setPower(.2);
                l_f_motor.setPower(.2);
                l_b_motor.setPower(-.2);
            }
        }
        else{
            while (eTime.time()<time){
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
    public void right()
    {
        strafe(true,.4);
        movement(getDistance(5.0),-.25);
        raiseArm(460, .2);
        releaseBlock();
        raiseArm(560,-.2);
        strafe(true,.5);

    }
    public void left()
    {
        strafe(true,4);
        movement(getDistance(5.0),-.25);
        raiseArm(460, .2);
        releaseBlock();
        raiseArm(560,-.2);
        strafe(false,.5);
    }
    public void center()
    {
        strafe(true,2);
        movement(getDistance(5.0),-.25);
        raiseArm(460, .2);
        releaseBlock();
        raiseArm(560,-.2);
    }

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() {
        initialize();
        //Get Vuforia stuff ready
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUOhDYf/////AAAAmdDp0b8XRUxwrHihX1jNl15KBIqZaXwn3EnrN6lJDn3BlYAco4a9MdghQnfu+QX+MUaAV3x8eOywibefAugDVdRRLjjYJGSNbljXvztRpkFYIrLZPfKGwHKibcGilsdTGiQebl4+lnGWCY4CzVx7lVFuXY6qWQ5lFAOGaJEUOHvQ1/R8iWUOyOg9RRXalnDsSsvvIoGN3cvqxq4xfcIb2r/Az+UQQTsi73p+GVzrYMDegDQ422tXTcSaLO5Kp6MA6+OfyuuF19nSsTd+5L6Zn3se9oVdsq+fcmffIzoVHaCVT+3rEVqQBngW+viXAvcHMATyVyZB2ZOG3sgBCQDpjPYZ0sKClH56Zj3x9RPnHFeI";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");


        ColorSensor sensorColor;
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        waitForStart();
        relicTrackables.activate();
        //start by closing the gripper on the block
        l_b_gripper.setPosition(0); //THESE NEED FIXING
        r_b_gripper.setPosition(0.35); //THESE VALUES NEED FIXING
        raiseArm(100,.2);
        flipGrip(true);
        //rotate the Linear Actuator to be parallel to the ground
        jewel_elbow.setPosition(1.0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        sleep(3000);




        if (sensorColor.red() > sensorColor.blue()) {
            //the color of the ball facing the sensor is red
            //we need to turn our arm to hit it off
            sleep(500);
            jewel_hand.setPosition(0);
            sleep(100);
            jewel_hand.setPosition(.5);
            sleep(100);
            jewel_hand.setPosition(0);
            sleep(100);
            jewel_hand.setPosition(.5);
            telemetry.addData("Color found is", "red");


        } else if (sensorColor.blue() > sensorColor.red()) {
            //the color of the ball facing the sensor is blue
            //we need to turn our arm to hit it off
            sleep(500);
            jewel_hand.setPosition(1);
            sleep(100);
            jewel_hand.setPosition(.5);
            sleep(100);
            jewel_hand.setPosition(1);
            sleep(100);
            jewel_hand.setPosition(.5);
            telemetry.addData("Color found is", "blue");


        } else //we find no color ball and we freak out internally - Peter
        {
            telemetry.addData("Color found is", "none lol git gud scrub");
            sleep(2000);
        }
        if (vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
            if (vuMark == RelicRecoveryVuMark.CENTER)
            {
                getOff();
                center();
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT)
            {
                getOff();
                right();
            }
            if (vuMark == RelicRecoveryVuMark.LEFT)
            {
                getOff();
                left();
            }
            telemetry.addData("VuMark", "%s visible", vuMark);

        }
        else {
            telemetry.addData("VuMark", "not visible");
            //prayer(eTime);
        }
        telemetry.update();




    }
}