/**
 * Created by pstevens on 11/22/17.
 * TODO
 *Make a calibration method using a limit switch to find the positions we need to make the servos go to
 *Using that, determine what the values have to be for the servos
 * Also waits must be replaced with another method of monitoring elapsed time
 */


/*import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import java.util.Arrays;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Nolan v2", group="TeleOp")
public class Nolan_V2_TeleOp extends OpMode {

	private static final double TRIGGERTHRESHOLD = .2;
	private static final double MINIMUM_STICK_INPUT_THRESHOLD = .15;
	private static final double SCALED_POWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train

	/* Safety limits: */
	/*private static final double __MINIMUM_LIFTER_POSITION = 0.2;
	private static final double __MAXIMUM_LIFTER_POSITION = 0.9;
	private static final int colorTolerance = 10;

	private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor;
	private static Servo l_gripper, r_gripper;
	private static DcMotor lifter_motor;
	private static Servo laArm, laElbow;

	private static boolean open_gripper;

	private double x; /* crabbing left and right */
	/*private double y; /* forward and reverse */
	//private double c; /* rotating clockwise and counter-clockwise */

	//@Override
	/*public void calib()
	{

	}
	public void init() {
     /*  MECANUM WHEEL AND MOTOR ARRANGEMENT:
     *
     *  LEFT       RIGHT
     *  \\\\--M  M--////
     *
     *
     *  ////--M  M--\\\\
     *
     */

		/*l_f_motor = hardwareMap.dcMotor.get("left_front");
		l_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
		l_b_motor = hardwareMap.dcMotor.get("left_back");
		l_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
		r_f_motor = hardwareMap.dcMotor.get("right_front");
		r_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
		r_b_motor = hardwareMap.dcMotor.get("right_back");
		r_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);

     /* LIFTER: */
		/*lifter_motor = hardwareMap.dcMotor.get("lift_motor");
		lifter_motor.setDirection(DcMotorSimple.Direction.REVERSE);

     /* GRIPPER: */
		/*l_gripper = hardwareMap.servo.get("left_gripper");
		r_gripper = hardwareMap.servo.get("right_gripper");

     /*Linear actuator servo and arm*/
		/*laElbow = hardwareMap.servo.get("la_elbow");
		laArm = hardwareMap.servo.get("la_arm");

		open_gripper = true;

		//not sure about these positions
		laElbow.setPosition(0.4);
		laArm.setPosition(0.0);
		l_gripper.setPosition(0.7);
		r_gripper.setPosition(0.3);

	}
	//currently the movement method relies on time, eventually we will want to use encoders
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
	public void autonomous()
	{
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
		l_gripper.setPosition(0.7);
		r_gripper.setPosition(0.3);
		//rotate the Linear Actuator to be parallel to the ground
		laElbow.setPosition(0.8);
		//extend the Linear Actuator
		laArm.setPosition(1); //may not be 1 depending on how far we actually will need to extend it
		//Now we attempt to read the color of the ball facing the sensor
		// convert the RGB values to HSV values.
		// multiply by the SCALE_FACTOR.
		// then cast it back to int (SCALE_FACTOR is a double)
		//got this from the template, not sure if I need it
		//This code assumes we are on BLUE team
		Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
				(int) (sensorColor.green() * SCALE_FACTOR),
				(int) (sensorColor.blue() * SCALE_FACTOR),
				hsvValues);
		if (Math.abs(245-sensorColor.red())<=colorTolerance)
		{
			//the color of the ball facing the sensor is red
			//we need to move forwards
			movement(.5, .5, .5, .5, eTime,  1);


		}
		else if (Math.abs(245-sensorColor.blue())<=colorTolerance)
		{
			//the color of the ball facing the sensor is blue
			//we need to move to the backwards
			movement(-.5, -0.5, -.5, -.5, eTime,  1);


		}
		//bring the linear actuator back to the initial position
		laArm.setPosition(0);
		laElbow.setPosition(.4);
		//strafe to the left about  inches
		//THIS ASSUMES WE ARE IN THE SPOT IN QUADRANT IV (quadrants numbered according to the coordinate plane)
		//strafe to the left
		movement(-.5, 0.5, 0.5, -0.5, eTime, 1.5);
		//start moving forward
		movement(.5, 0.5, .5, .5, eTime, 4);
		//open the grippers to release the block
		l_gripper.setPosition(0.7);
		r_gripper.setPosition(0.3);
		movement(-.1,-.1,-.1,-.1, eTime, 2);
		//close the grippers
		l_gripper.setPosition(0.7);
		r_gripper.setPosition(0.3);
		movement(.1,.1,.1,.1, eTime, 2);
		//back up to not be touching the glyph
		movement(-.1,-.1,-.1,-.1, eTime,1);

	}
	@Override
	public void loop() {
     /* Sample the X and Y position of the gamepad and if the input is less than minimum threshold,
     * ignore it in order to prevent robot jitter.  We sample each input value only once to
     * avoid sampling the input again and get different value if the stick have moved. */
		/* y = gamepad1.left_stick_y;
		if (Math.abs(y) < MINIMUM_STICK_INPUT_THRESHOLD) y = 0;

		x = -gamepad1.left_stick_x;
		if (Math.abs(x) < MINIMUM_STICK_INPUT_THRESHOLD) x = 0;

		c = -gamepad1.right_stick_x;
		if (Math.abs(c) < MINIMUM_STICK_INPUT_THRESHOLD) c = 0;

		double l_f = y + x + c;
		double r_f = y - x - c;
		double l_b = y - x + c;
		double r_b = y + x - c;

     /* figure out how much power to put to each wheel */
		/*/double[] wheel_power = {l_f, r_f, l_b, r_b};
		double maximum_power;
		Arrays.sort(wheel_power);
		maximum_power = wheel_power[3];
		if (maximum_power > 1.0) { /* scale wheel power proportionally */
		/*	l_f /= maximum_power;
			r_f /= maximum_power;
			l_b /= maximum_power;
			r_b /= maximum_power;
		}
		l_f_motor.setPower(l_f * SCALED_POWER);
		r_f_motor.setPower(r_f * SCALED_POWER);
		l_b_motor.setPower(l_b * SCALED_POWER);
		r_b_motor.setPower(r_b * SCALED_POWER);

     /* After we are finished with driving the platform, look at the position of the lift platform and gripper. */
		/*double lifter_input = gamepad2.left_stick_y;
		if (Math.abs(lifter_input) < MINIMUM_STICK_INPUT_THRESHOLD) lifter_input = 0;
		lifter_motor.setPower(lifter_input);

		float l_gripper_input = gamepad2.left_trigger;
		if ( l_gripper_input > 0.5 ) open_gripper = false;

		float r_gripper_input = gamepad2.right_trigger;
		if ( r_gripper_input > 0.5 ) open_gripper = true;

		if ( open_gripper ) {
			l_gripper.setPosition(0.7);
			r_gripper.setPosition(0.3);
		}
		else {
			l_gripper.setPosition(0.2);
			r_gripper.setPosition(0.8);
		}
	}
}

/* tested checkin */
