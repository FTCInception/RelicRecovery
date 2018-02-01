package org.firstinspires.ftc.teamcode.nolan;
/**
 * Created by pstevens on 11/22/17.
 */

/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="NolanTeleOpV3", group="TeleOp")
public class Nolan_V3TeleOp extends OpMode {

    private static final double TRIGGERTHRESHOLD = .2;
    private static final double ACCEPTINPUTTHRESHOLD = .15;
    private static final double SCALEDPOWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train

    private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static CRServo l_b_gripper, r_b_gripper, l_t_gripper, r_t_gripper;
    private static Servo gripperFlipper, relicGripper, relicTooth;
    private static DcMotor lifter_motor, relic_motor;
    private static Servo jewel_hand, jewel_elbow;
    private static boolean open_gripper;

    @Override
    public void init() {
        l_f_motor = hardwareMap.dcMotor.get("left_front");
        l_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        l_b_motor = hardwareMap.dcMotor.get("left_back");
        l_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_f_motor = hardwareMap.dcMotor.get("right_front");
        r_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        r_b_motor = hardwareMap.dcMotor.get("right_back");
        r_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //NOTE In the example the DcMotorController.RunMode was used, but this wasn't working so I changed it to DcMotor.RunMode

        //Relic
        relic_motor = hardwareMap.dcMotor.get("relic motor");
        relic_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        relicGripper = hardwareMap.servo.get("relic_arm");
        relicTooth = hardwareMap.servo.get("relic_tooth");


   /* Arm: */
        lifter_motor = hardwareMap.dcMotor.get("intake_arm");
        lifter_motor.setDirection(DcMotorSimple.Direction.REVERSE);

   /* GRIPPERS: */
        l_b_gripper = hardwareMap.crservo.get("left_bottom_arm");
        r_b_gripper = hardwareMap.crservo.get("right_bottom_arm");
        l_t_gripper = hardwareMap.crservo.get("left_top_arm");
        r_t_gripper = hardwareMap.crservo.get("right_top_arm");


   /*Jewel servo and arm*/
        jewel_elbow = hardwareMap.servo.get("la_elbow");
        jewel_hand = hardwareMap.servo.get("la_arm");

        open_gripper = true;

        //not sure about these positions
        jewel_elbow.setPosition(0.4);
        jewel_hand.setPosition(0.5);
        gripperFlipper.setPosition(.9);

    }

    @Override
    public void loop() {
        double inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y : 0;
        double inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x : 0;
        double inputC = Math.abs(gamepad1.right_stick_x)> ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x: 0;



        double relic_arm_input = 0;
        double power_left = -(gamepad1.left_trigger);
        double power_right = gamepad1.right_trigger;
        double inputLift = gamepad2.left_stick_y;
        boolean relicarm = gamepad2.dpad_up;
        boolean reverseRelicarm = gamepad2.dpad_down;
        boolean toothBite = gamepad2.b;
        boolean toothRelease = gamepad2.x;
        boolean relicWristUp = gamepad2.y;
        boolean relicWristDown = gamepad2.a;
        boolean gripperflip = gamepad2.right_stick_button;
        boolean gripperflipreverse = gamepad2.left_stick_button;
        boolean outtake = gamepad1.left_bumper;


        boolean servostaystill = gamepad2.b;




        arcadeMecanum(inputY, inputX, inputC, l_f_motor, r_f_motor, l_b_motor, r_b_motor, inputLift, lifter_motor, l_b_gripper, r_b_gripper, l_t_gripper, r_t_gripper, power_left, power_right, relic_arm_input, relicarm, relic_motor, toothRelease, relicWristUp, relicWristDown, toothBite, gripperflip, gripperFlipper, outtake, jewel_elbow, jewel_hand, reverseRelicarm, gripperflipreverse);
    }



    // y - forwards
    // x - side
    // c - rotation
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double inputLift, DcMotor lifter_motor, CRServo l_b_gripper, CRServo r_b_gripper, CRServo l_t_gripper, CRServo r_t_gripper, double power_left, double power_right, double relic_arm_input, boolean relicarm, DcMotor relic_motor, boolean toothRelease, boolean relicWristUp, boolean relicWristDown, boolean toothBite, boolean gripperflip, Servo gripperFlipper, boolean outtake, Servo jewel_elbow, Servo jewel_hand, boolean reverseRelicarm, boolean flipgripreverse) {
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

        relic_motor.setPower(relic_arm_input);

        lifter_motor.setPower(inputLift/2);

        l_b_gripper.setPower(power_left);
        l_t_gripper.setPower(-power_left);
        r_b_gripper.setPower(power_right);
        r_t_gripper.setPower(-power_right);

        if(outtake){
            l_b_gripper.setPower(-power_left);
            l_t_gripper.setPower(-power_left);
            r_b_gripper.setPower(-power_right);
            r_t_gripper.setPower(-power_right);
        }

        if (toothBite){
            relicTooth.setPosition(0);
        }
        else if (toothRelease){
            relicTooth.setPosition(1);
        }
        if (relicWristDown){
            relicGripper.setPosition(.1);
        }
        else if (relicWristUp){
            relicGripper.setPosition(.6);
        }
        if (gripperflip){
            gripperFlipper.setPosition(0);
        }
        else if(flipgripreverse){
            gripperFlipper.setPosition(.8);
        }
        if (relicarm){
            relic_arm_input  = 1;
        }

        else if(reverseRelicarm){
            relic_arm_input = -1;
        }
        else{
            relic_arm_input = 0;
        }








    }
}