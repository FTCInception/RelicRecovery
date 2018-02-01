

package org.firstinspires.ftc.teamcode.nolan;
/**
 * Created by pstevens on 11/22/17.
 * CODE FOR QUADRANT III (Red)
 */

/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
import android.app.Activity;
import android.view.View;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.
If you use our code and see us at competition, come say hello!
*/

//@com.qualcomm.robotcore.eventloop.  opmode.TeleOp(name="Nolan v3", group="TeleOp")
@Disabled
@Autonomous(name="Blue_Position_FAR", group="Autonomous")

public class AutonomousV2 extends LinearOpMode {

    private static final double TRIGGERTHRESHOLD = .2;
    private static final double MINIMUM_STICK_INPUT_THRESHOLD = .15;
    private static final double SCALED_POWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train

    /* Safety limits: */
    private static final double __MINIMUM_LIFTER_POSITION = 0.2;
    private static final double __MAXIMUM_LIFTER_POSITION = 0.9;
    private static final int colorTolerance = 30;//subject to change

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
        laArm = hardwareMap.servo.get("la_arm");

        open_gripper = true;

        //not sure about these positions
        laElbow.setPosition(0.4);
        laArm.setPosition(0.0);
        l_gripper.setPosition(0.4);
        r_gripper.setPosition(0.55);
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
        l_gripper.setPosition(0.0);
        r_gripper.setPosition(0.7);

        //rotate the Linear Actuator to be parallel to the ground
        laElbow.setPosition(1.0);
        //Now we attempt to read the color of the ball facing the sensor
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        //got this from the template, not sure if I need it
        //This code assumes we are on BLUE team
        eTime.reset();
        while (eTime.time()< 10){
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
        }

        //THIS WAS NOT PERFORMED IN THE LAST TEST
        //I manipulated color tolerance, mess around with it to see if it works
        if (Math.abs(245-sensorColor.red())<=colorTolerance)
        {
            //the color of the ball facing the sensor is red
            //we need to move forwards
            movement(.1, .1, .1, .1, eTime,  .75);


        }
        else if (Math.abs(245-sensorColor.blue())<=colorTolerance)
        {
            //the color of the ball facing the sensor is blue
            //we need to move to the backwards
            movement(-.1, -0.1, -.1, -.1, eTime, .75);


        }
        //bring the arm back to the initial position
        laElbow.setPosition(.5);

        //THIS ASSUMES WE ARE IN THE SPOT IN QUADRANT III (quadrants numbered according to the coordinate plane)
        //strafe to the left
        movement(.1, -0.1, -0.1, 0.1, eTime, 1.5);
        //start moving forward
        movement(.1, 0.1, .1, .1, eTime, 1);
        //Turn around
        movement(.1,.1,-.1,-.1,eTime,1.5);
        movement(.1, 0.1, .1, .1, eTime, .75);
        //open the grippers to release the block
        l_gripper.setPosition(0.4);
        r_gripper.setPosition(0.55);
        //Push the block in
        movement(.1, 0.1, .1, .1, eTime, .5);
        movement(-.1,-.1,-.1,-.1, eTime, .2);

      }
}