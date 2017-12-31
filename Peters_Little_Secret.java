package org.firstinspires.ftc.teamcode; /**
 * Created by pstevens on 11/22/17.
 */

/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/*
This code is written as an example only.
Obviously, it was not tested on your team's robot.
Teams who use and reference this code are expected to understand code they use.

If you use our code and see us at competition, come say hello!
*/

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="In Case of Emergency(!!!)", group="TeleOp")
public class Peters_Little_Secret extends OpMode {

    private static final double TRIGGERTHRESHOLD = .2;
    private static final double ACCEPTINPUTTHRESHOLD = .15;
    private static final double SCALEDPOWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train

    private static DcMotor left_front_wheel, left_back_wheel, right_front_wheel, right_back_wheel;
    private static Servo left_servo,right_servo;
    private static DcMotor lift_motor;
    private static Servo jewel_arm, jewel_hand;

    private static boolean gripper_is_open;

    @Override
    public void init() {
        left_front_wheel = hardwareMap.dcMotor.get("left_front");
        left_back_wheel = hardwareMap.dcMotor.get("left_back");
        right_front_wheel = hardwareMap.dcMotor.get("right_front");
        right_back_wheel = hardwareMap.dcMotor.get("right_back");
        right_back_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        ;

        // gripper
        lift_motor = hardwareMap.dcMotor.get("lift");
        left_servo = hardwareMap.servo.get("left_arm");
        right_servo = hardwareMap.servo.get("right_arm");
        jewel_arm = hardwareMap.servo.get("la_elbow");
        jewel_hand = hardwareMap.servo.get("la_arm");

        jewel_arm.setPosition(.4);
        jewel_hand.setPosition(.5);
        gripper_is_open = true;
        left_servo.setPosition(0.4); // open
        right_servo.setPosition(0.15); // open
        // top_right_grip.setPosition(0.8); // closed
        //bottom_left_grip.setPosition(0.6); // open
        // bottom_left_grip.setPosition(0.1); // closed
        //bottom_right_grip.setPosition(0.3); // open
//      double volts = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
    }

    @Override
    public void loop() {
        double inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y : 0;
        double inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x : 0;
        double inputC = Math.abs(gamepad1.right_stick_x)> ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x: 0;

        double inputLift = gamepad2.left_stick_y;
        float inputGripperOpen = gamepad1.left_trigger;
        float inputGripperClose = gamepad1.right_trigger;
        boolean servostaystill = gamepad2.b;

        if ( inputGripperClose > 0.5 ) gripper_is_open = false;
        if ( inputGripperOpen > 0.5 ) gripper_is_open = true;

        arcadeMecanum(inputY, inputX, inputC, left_front_wheel, right_front_wheel, left_back_wheel, right_back_wheel, inputLift, lift_motor, left_servo, right_servo, jewel_arm, jewel_hand, servostaystill);
    }



    // y - forwards
    // x - side
    // c - rotation
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double inputLift, DcMotor lift_motor, Servo left_servo, Servo right_servo, Servo jewel_arm, Servo jewel_hand, boolean servostaystill) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        double scaledPower = SCALEDPOWER;

        leftFront.setPower(leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower));
        rightFront.setPower(rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower));
        leftBack.setPower(leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower));
        rightBack.setPower(rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower));

        lift_motor.setPower(inputLift);

        if (servostaystill){
            jewel_arm.setPosition(.4);
            jewel_hand.setPosition(.5);
        }


        if ( gripper_is_open ) {
            left_servo.setPosition(.2);
            right_servo.setPosition(.4);
        }
        else {
            left_servo.setPosition(0);
            right_servo.setPosition(.6);
        }
    }
}