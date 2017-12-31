
package org.firstinspires.ftc.teamcode;
/**
 * Created by pstevens on 11/22/17.
 */

/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/* Peter code revision - 12/4/17
Looked at Nathaniel's existing code and made modifications the key one being the lack of linear actuator
use, as we many not be able to use that at this upcoming competition (Help it's 5 days we're so not prepared)
Also include telemetry statements while reading the jewels, so as to determine what color we're actually reading.
Finally, I added a backup in case we read no colored jewel where we just pick the jewel arm back up, rather than not doing anything
 */

//@com.qualcomm.robotcore.eventloop.  opmode.TeleOp(name="Nolan v3", group="TeleOp")
@Disabled
@Autonomous(name="Blue_Position_FAR", group="Quadrant IV")

public class Nathaniel_BAD_CODE extends LinearOpMode {

    private static final double TRIGGERTHRESHOLD = .2;
    private static final double MINIMUM_STICK_INPUT_THRESHOLD = .15;
    private static final double SCALED_POWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train

    /* Safety limits: */
    private static final double __MINIMUM_LIFTER_POSITION = 0.2;
    private static final double __MAXIMUM_LIFTER_POSITION = 0.9;
    private static final int colorTolerance = 10;

    private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static Servo l_gripper, r_gripper;
    private static DcMotor lifter_motor;
    private static Servo laArm, laElbow;

    private static boolean open_gripper;

    private double x; /* crabbing left and right */
    private double y; /* forward and reverse */
    private double c; /* rotating clockwise and counter-clockwise */


    public void initialize() {
        l_f_motor = hardwareMap.dcMotor.get("left_front");
        l_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        l_b_motor = hardwareMap.dcMotor.get("left_back");
        l_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_f_motor = hardwareMap.dcMotor.get("right_front");
        r_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        r_b_motor = hardwareMap.dcMotor.get("right_back");
        r_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);

    /* LIFTER: */
        lifter_motor = hardwareMap.dcMotor.get("lift");
        lifter_motor.setDirection(DcMotorSimple.Direction.REVERSE);

    /* GRIPPER: */
        l_gripper = hardwareMap.servo.get("left_arm");
        r_gripper = hardwareMap.servo.get("right_arm");

    /*Linear actuator servo and arm*/
        laElbow = hardwareMap.servo.get("la_elbow");
        //laArm = hardwareMap.servo.get("la_arm");

        open_gripper = true;

        //not sure about these positions
        laElbow.setPosition(0.4);
        laArm.setPosition(0.5);
        l_gripper.setPosition(0.4);
        r_gripper.setPosition(0.3);
    }
    //Currently the movement method relies on time, eventually we will want to switch to using motor encoders
    public void movement(double rf, double rb, double lf, double lb, ElapsedTime eTime, double time) {
        eTime.reset();
        while (eTime.time()< time){
            r_f_motor.setPower(rf);
            r_b_motor.setPower(rb);
            l_f_motor.setPower(lf);
            l_b_motor.setPower(lb);
        }

        r_f_motor.setPower(0);
        r_b_motor.setPower(0);
        l_f_motor.setPower(0);
        l_b_motor.setPower(0);
    }
    @Override
    public void runOpMode()
    {
        initialize();
        ElapsedTime eTime = new ElapsedTime();
        ColorSensor sensorColor;
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        // wait for the start button to be pressed.
        waitForStart();
        //start by closing the gripper on the block
        l_gripper.setPosition(0);
        r_gripper.setPosition(0.6);
        //rotate the Linear Actuator to be parallel to the ground
        laElbow.setPosition(1.0);
        //extend the Linear Actuator
        //laArm.setPosition(0); //may not be 1 depending on how far we actually will need to extend it
        //Now we attempt to read the color of the ball facing the sensor
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        //got this from the template, not sure if I need it
        //This code assumes we are on BLUE team
        /*eTime.reset();
        while (eTime.time()< 5) {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
        }*/

        if (sensorColor.red()>sensorColor.blue())
        {
            //the color of the ball facing the sensor is red
            //we need to turn our arm to hit it off
            laArm.setPosition(0);
            telemetry.addData("Color found is","red");
            sleep(3000);


        }
        else if (sensorColor.blue()>sensorColor.red())
        {
            //the color of the ball facing the sensor is blue
            //we need to move to the backwards
            laArm.setPosition(1);
            telemetry.addData("Color found is","blue");
            sleep(3000);


        }
        else //we find no color ball and we freak out internally - Peter
        {
            //laElbow.setPosition(.);
            telemetry.addData("Color found is", "none lol git gud scrub");
            sleep(2000);
        }
        //bring the linear actuator back to the initial position
        laArm.setPosition(.5);
        laElbow.setPosition(.4);
        //strafe to the left about  inches
        //THIS ASSUMES WE ARE IN THE SPOT IN QUADRANT IV (quadrants numbered according to the coordinate plane)
        //strafe to the left
        lifter_motor.setPower(.5); //move lift slightly up, this might be more than slightly though - needs testing - Peter
        sleep(100);
        //movement(-.25, 0.25, 0.25, -0.25, eTime, 1.5); - I have a feeling this line may not be needed(?) We'll have to test it though- Peter
        //start moving forward
        movement(-.25, -0.25, -.25, -.25, eTime, 2);
        //strafe right
        movement(.25, -0.25, -0.25, 0.25, eTime, 1.5);
        movement(0, 0, 0, 0, eTime, .5);
        //open the grippers to release the block
        l_gripper.setPosition(0.4);
        r_gripper.setPosition(0.3);
        movement(-.15,-.15,-.15,-.15, eTime, 2); //Made this the glyph-shove (official term) - Peter
        //close the grippers
        //l_gripper.setPosition(0.7);
        //r_gripper.setPosition(0.3);
        //movement(.15,.15,.15,.15, eTime, 2); - Don't need this anymore due to aforementioned glyph-shove (still official term) - Peter
        //back up to not be touching the glyph
        movement(.15,.15,.15,.15, eTime,1);

    }
}