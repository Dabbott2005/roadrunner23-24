package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="BasicLeftAuto")
public class BasicLeftAuto extends LinearOpMode {
        private DcMotor frontLeft = null;
        private DcMotor frontRight = null;
        private DcMotor backLeft = null;
        private DcMotor backRight = null;
        private DcMotorEx gripRotation = null;
        private DcMotorEx leftSlide = null;
        private DcMotorEx rightSlide = null;
         //Servos
         private Servo depositServo = null;
         private Servo liftServo = null;
         private Servo clawServo = null;
         private Servo planeServo = null;
         private Servo holdServo = null;



        @Override
        public void runOpMode() {

            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            gripRotation = hardwareMap.get(DcMotorEx.class, "griprotation");
            leftSlide = hardwareMap.get(DcMotorEx.class, "leftslide");
            rightSlide = hardwareMap.get(DcMotorEx.class, "rightslide");


            depositServo = hardwareMap.get(Servo.class, "depositservo");
            liftServo = hardwareMap.get(Servo.class, "liftservo");
            clawServo = hardwareMap.get(Servo.class, "clawservo");
            planeServo = hardwareMap.get(Servo.class, "planeservo");
            holdServo = hardwareMap.get(Servo.class, "holdservo");

            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            waitForStart();
                clawServo.setPosition(1);
                gripRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gripRotation.setTargetPosition(-1000);
                gripRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (!(gripRotation.getCurrentPosition() == gripRotation.getTargetPosition())) {
                    gripRotation.setPower(.4);
                }
                gripRotation.setPower(0);

                backLeft.setTargetPosition(1000);
                backRight.setTargetPosition(1000);
                frontLeft.setTargetPosition(1000);
                frontRight.setTargetPosition(1000);//move forward

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(.7);
                    backRight.setPower(.7);
                    frontLeft.setPower(.7);
                    frontRight.setPower(.7);
                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(1000);

                clawServo.setPosition(0);

                sleep(1000);


        }
}
