package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DriverNew", group="Linear OpMode")

public class Teleop_NEW extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor Slide = null;

    private DcMotor Intake = null;

    private DcMotor liftLeft = null;

    private DcMotor liftRight = null;


    //Servos
    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo angleServo = null;
    private Servo planeServo = null;

    private Servo hangServo = null;




    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");


        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        planeServo = hardwareMap.get(Servo.class, "planeservo");
        hangServo = hardwareMap.get(Servo.class,"hangServo");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));//

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
            if(gamepad1.left_trigger>0.1) {
                leftFrontPower  /= 2;
                rightFrontPower /= 2;
                leftBackPower   /= 2;
                rightBackPower  /= 2;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);



            // Designate buttons
            while(gamepad1.b) {
                planeServo.setPosition(.5);
                sleep(500);
                planeServo.setPosition(0);
            }
            while(gamepad1.y){
                hangServo.setPosition(0.1);
            }
            while (gamepad1.x) {
                hangServo.setPosition(.5);
            }

            if (gamepad2.a) {
                Intake.setPower(-1);

            }
            if (gamepad2.b) {
                Intake.setPower(.5);
            }
            if ((!gamepad2.b) && (!gamepad2.a)) {
                Intake.setPower(0);
            }

            if(gamepad2.y) {
                leftServo.setPosition(1);
                rightServo.setPosition(0);
                Intake.setPower(-.4);
            }

            if((!gamepad2.y) && (!gamepad2.right_bumper) && (!gamepad2.left_bumper)) {
                leftServo.setPosition(.5);
                rightServo.setPosition(.5);
            }

            if(gamepad2.right_bumper) {
                rightServo.setPosition(0);
            }
            if(gamepad2.left_bumper) {
                leftServo.setPosition(1);
            }
            if(gamepad2.dpad_up) {
                angleServo.setPosition(.5); // ground level
            }
            if(gamepad2.dpad_down) {
                angleServo.setPosition(.1);
            }
            if (gamepad2.dpad_right) {
                angleServo.setPosition(.2);
            }
            if(gamepad2.dpad_left) {
                angleServo.setPosition(.4);
            }


            Slide.setPower(gamepad2.left_stick_y);
            liftLeft.setPower(gamepad2.right_stick_y);
            liftRight.setPower(gamepad2.right_stick_y);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}

