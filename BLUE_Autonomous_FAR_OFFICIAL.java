package org.firstinspires.ftc.teamcode.nolan;
/**
 * Created by NPlaxton on 12/22/17.
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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//@com.qualcomm.robotcore.eventloop.  opmode.TeleOp(name="Nolan v3", group="TeleOp")
@Autonomous(name="Blue_Position_FAR", group="BLUE_FAR")

public class BLUE_Autonomous_FAR_OFFICIAL extends LinearOpMode {


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
        while (eTime.time() < time && opModeIsActive()) {
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
    public void right(ElapsedTime eTime)
    {

    }
    public void left(ElapsedTime eTime)
    {

    }
    public void center(ElapsedTime eTime)
    {

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

        ElapsedTime eTime = new ElapsedTime();
        ColorSensor sensorColor;
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        waitForStart();
        relicTrackables.activate();
        //start by closing the gripper on the block
        l_gripper.setPosition(0);
        r_gripper.setPosition(0.35);
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
                center(eTime);
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT)
            {
                right(eTime);
            }
            if (vuMark == RelicRecoveryVuMark.LEFT)
            {
                left(eTime);
            }
            telemetry.addData("VuMark", "%s visible", vuMark);

        }
        else {
            telemetry.addData("VuMark", "not visible");
            prayer(eTime);
        }
        telemetry.update();




    }
}