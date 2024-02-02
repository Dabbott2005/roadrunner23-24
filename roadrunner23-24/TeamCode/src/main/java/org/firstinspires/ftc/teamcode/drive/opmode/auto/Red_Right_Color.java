package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name = "Red_Right_PR")
public class Red_Right_Color extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
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
    OpenCvWebcam webcam1;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");


        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        planeServo = hardwareMap.get(Servo.class, "planeservo");
        hangServo = hardwareMap.get(Servo.class,"hangServo");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);



        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        RedPropDectector detector = new RedPropDectector(telemetry);
        webcam1.setPipeline(detector);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        switch (detector.getLocation()) {
            case LEFT:
                leftServo.setPosition(.5);
                rightServo.setPosition(.5);
                angleServo.setPosition(.4);
                hangServo.setPosition(.5);

                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                liftLeft.setTargetPosition(3100);
                liftRight.setTargetPosition(3100);

                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftLeft.setPower(1);
                liftRight.setPower(1);
                //while (!(liftRight.getCurrentPosition() == liftRight.getTargetPosition())) {}
                while (liftLeft.isBusy()){
                    telemetry.addData("Lift at", liftLeft.getCurrentPosition());
                    telemetry.update();
                }
                liftLeft.setPower(0);
                liftRight.setPower(0);


                backLeft.setTargetPosition(400);
                backRight.setTargetPosition(400);
                frontLeft.setTargetPosition(400);
                frontRight.setTargetPosition(400);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setPower(.5);
                backRight.setPower(.5);
                frontLeft.setPower(.5);
                frontRight.setPower(.5);

                while (frontRight.isBusy()) {

                }

                backLeft.setTargetPosition(-890);
                backRight.setTargetPosition(890);
                frontLeft.setTargetPosition(890);
                frontRight.setTargetPosition(-890);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (frontRight.isBusy()) {

                }

                backLeft.setTargetPosition(300);
                backRight.setTargetPosition(300);
                frontLeft.setTargetPosition(300);
                frontRight.setTargetPosition(300);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (frontRight.isBusy()) {

                }


                backLeft.setTargetPosition(-410);
                backRight.setTargetPosition(410);
                frontLeft.setTargetPosition(-410);
                frontRight.setTargetPosition(410);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (frontRight.isBusy()) {

                }

                backLeft.setTargetPosition(120);
                backRight.setTargetPosition(120);
                frontLeft.setTargetPosition(120);
                frontRight.setTargetPosition(120);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (frontRight.isBusy()) {

                }

                sleep(1000);

                angleServo.setPosition(0);
                sleep(1000);

                leftServo.setPosition(1);
                rightServo.setPosition(.5);

                sleep(2000);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setTargetPosition(200);
                backRight.setTargetPosition(200);
                frontLeft.setTargetPosition(200);
                frontRight.setTargetPosition(200);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (frontRight.isBusy()) {

                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



                backLeft.setTargetPosition(-200);
                backRight.setTargetPosition(-200);
                frontLeft.setTargetPosition(-200);
                frontRight.setTargetPosition(-200);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (frontRight.isBusy()) {

                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);


                backLeft.setTargetPosition(1100);
                backRight.setTargetPosition(-1100);
                frontLeft.setTargetPosition(1100);
                frontRight.setTargetPosition(-1100);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (frontRight.isBusy()) {

                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);


                backLeft.setTargetPosition(220);
                backRight.setTargetPosition(220);
                frontLeft.setTargetPosition(220);
                frontRight.setTargetPosition(220);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (frontRight.isBusy()) {

                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                backLeft.setTargetPosition(-100);
                backRight.setTargetPosition(100);
                frontLeft.setTargetPosition(100);
                frontRight.setTargetPosition(-100);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (frontRight.isBusy()) {

                }

                sleep(1000);

                Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Slide.setTargetPosition(-1550);


                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slide.setPower(1);

                while (Slide.isBusy()) {

                }

                rightServo.setPosition(0);

                sleep(2000);

                Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Slide.setTargetPosition(800);


                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slide.setPower(1);

                while (Slide.isBusy()) {

                }
                Slide.setPower(0);


                backLeft.setTargetPosition(-700);
                backRight.setTargetPosition(700);
                frontLeft.setTargetPosition(700);
                frontRight.setTargetPosition(-700);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (frontRight.isBusy()) {

                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                break;

            case MIDDLE:

                break;
            case RIGHT:

                break;
        }
        webcam1.stopStreaming();

    }
}
