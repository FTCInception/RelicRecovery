
package org.firstinspires.ftc.teamcode;
/**
 * Created by pstevens on 11/22/17.
 */

/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

/* Peter code revision - 12/8/17
Took the previously revised code and removed all unnecessary lines, also added changed values to reflect changes to TeleOp.
Main change is the use of the statement if(red > blue), rather than using hsValues and limits that I don't quite understand.
This proved to actually detect color when at Vandegrift. I also added the use of a servo gimble,(something that makes the jewel arm
move by itself) for a better auto.
 */

//@com.qualcomm.robotcore.eventloop.  opmode.TeleOp(name="Nolan v3", group="TeleOp")
@Autonomous(name="RED_Autonomous_NEAR", group="RED_NEAR")

public class RED_Autonomous_NEAR_OFFICIAL extends LinearOpMode {


    private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static Servo l_gripper, r_gripper;
    private static DcMotor lifter_motor;
    private static Servo jewel_hand, jewel_elbow;

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

    /* LIFTER: */
        lifter_motor = hardwareMap.dcMotor.get("lift");
        lifter_motor.setDirection(DcMotorSimple.Direction.REVERSE);

    /* GRIPPER: */
        l_gripper = hardwareMap.servo.get("left_arm");
        r_gripper = hardwareMap.servo.get("right_arm");

    /*Linear actuator servo and arm*/
        jewel_elbow = hardwareMap.servo.get("la_elbow");
        jewel_hand = hardwareMap.servo.get("la_arm");

        open_gripper = true;

        //not sure about these positions
        jewel_elbow.setPosition(0.4);
        jewel_hand.setPosition(0.5);
        l_gripper.setPosition(0.4);
        r_gripper.setPosition(0.1);
    }
    //Currently the movement method relies on time, eventually we will want to switch to using motor encoders
    public void movement(double rf, double rb, double lf, double lb, ElapsedTime eTime, double time) {
        eTime.reset();
        while (eTime.time()< time && opModeIsActive()){
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
        waitForStart();
        //start by closing the gripper on the block
        l_gripper.setPosition(0);
        r_gripper.setPosition(0.6);
        //rotate sensor to be in between jewels
        jewel_elbow.setPosition(1.0);
        sleep(3000);

        if (sensorColor.blue()>sensorColor.red())
        {
            //the color of the ball facing the sensor is blue
            //we need to turn our arm to hit it off
            sleep(500);
            jewel_hand.setPosition(0);
            sleep(100);
            jewel_hand.setPosition(.5);
            sleep(100);
            jewel_hand.setPosition(0);
            sleep(100);
            jewel_hand.setPosition(.5);
            telemetry.addData("Color found is","red");


        }
        else if (sensorColor.red()>sensorColor.blue())
        {
            //the color of the ball facing the sensor is red
            //we need to turn our arm to hit it off
            sleep(500);
            jewel_hand.setPosition(1);
            sleep(100);
            jewel_hand.setPosition(.5);
            sleep(100);
            jewel_hand.setPosition(1);
            sleep(100);
            jewel_hand.setPosition(.5);
            telemetry.addData("Color found is","blue");


        }
        else //we find no color ball and we freak out internally - Peter
        {
            telemetry.addData("Color found is", "none lol git gud scrub");
            sleep(2000);
        }
        sleep(1000);
        jewel_hand.setPosition(.5);
        jewel_elbow.setPosition(.4);
        lifter_motor.setPower(.5); //move lift slightly up, this might be more than slightly though - needs testing - Peter
        sleep(500);
        movement(.25,.25,.25,.25, eTime,2);
        movement(0, 0, 0, 0, eTime, .1);
        movement(.25,.25,-.25,-.25, eTime,2.5);
        movement(-.25, 0.25, 0.25, -0.25, eTime, 1.5);
        //open the grippers to release the block
        l_gripper.setPosition(0.2);
        r_gripper.setPosition(0.4);
        movement(-.15,-.15,-.15,-.15, eTime, 1); //Made this the glyph-shove (official term) - Peter
        //back up to not be touching the glyph
        movement(.15,.15,.15,.15, eTime,1);

    }
}