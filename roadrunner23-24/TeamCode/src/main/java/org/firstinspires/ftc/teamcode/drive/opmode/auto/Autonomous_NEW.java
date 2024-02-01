package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="redRightNEW")
public class Autonomous_NEW extends LinearOpMode {
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


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "redProp.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/redProp.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "redProp",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private List<Recognition> lastRecognitions = new ArrayList<>();


    @Override
    public void runOpMode() {

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



        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to get the dub");
        telemetry.update();


        waitForStart();

        if (opModeIsActive()) {


            telemetryTfod();


            if (tfod != null) {
                tfod.shutdown();
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {


        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.65f);


        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getFreshRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        if (currentRecognitions.size()<1) {
            //RIGHT
            leftServo.setPosition(.5);
            rightServo.setPosition(.5);
            angleServo.setPosition(.4);
            planeServo.setPosition(-1);
            hangServo.setPosition(.5);

            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            liftLeft.setTargetPosition(3100);
            liftRight.setTargetPosition(3100);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!(liftRight.getCurrentPosition() == liftRight.getTargetPosition())) {

                liftLeft.setPower(1);
                liftRight.setPower(1);

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


            while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                backLeft.setPower(.5);
                backRight.setPower(.5);
                frontLeft.setPower(.5);
                frontRight.setPower(.5);
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);


            backLeft.setTargetPosition(-890);
            backRight.setTargetPosition(890);
            frontLeft.setTargetPosition(890);
            frontRight.setTargetPosition(-890);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                backLeft.setPower(-.7);
                backRight.setPower(.7);
                frontLeft.setPower(.7);
                frontRight.setPower(-.7);
            }
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);


            backLeft.setTargetPosition(300);
            backRight.setTargetPosition(300);
            frontLeft.setTargetPosition(300);
            frontRight.setTargetPosition(300);

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


            backLeft.setTargetPosition(-410);
            backRight.setTargetPosition(410);
            frontLeft.setTargetPosition(-410);
            frontRight.setTargetPosition(410);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                backLeft.setPower(-.7);
                backRight.setPower(.7);
                frontLeft.setPower(-.7);
                frontRight.setPower(.7);
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);


            backLeft.setTargetPosition(120);
            backRight.setTargetPosition(120);
            frontLeft.setTargetPosition(120);
            frontRight.setTargetPosition(120);

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



            backLeft.setTargetPosition(-200);
            backRight.setTargetPosition(-200);
            frontLeft.setTargetPosition(-200);
            frontRight.setTargetPosition(-200);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                backLeft.setPower(-.7);
                backRight.setPower(-.7);
                frontLeft.setPower(-.7);
                frontRight.setPower(-.7);
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


            while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                backLeft.setPower(-.7);
                backRight.setPower(.7);
                frontLeft.setPower(-.7);
                frontRight.setPower(.7);
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

            backLeft.setTargetPosition(-100);
            backRight.setTargetPosition(100);
            frontLeft.setTargetPosition(100);
            frontRight.setTargetPosition(-100);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                backLeft.setPower(-.4);
                backRight.setPower(-.4);
                frontLeft.setPower(-.4);
                frontRight.setPower(-.4);
            }
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
             sleep(1000);

            Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Slide.setTargetPosition(-1550);


            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            while (!(Slide.getCurrentPosition() == Slide.getTargetPosition())) {

                Slide.setPower(1);

            }
            Slide.setPower(0);

            rightServo.setPosition(0);

            sleep(2000);

            Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Slide.setTargetPosition(800);


            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            while (!(Slide.getCurrentPosition() == Slide.getTargetPosition())) {

                Slide.setPower(-1);

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


            while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                backLeft.setPower(-.7);
                backRight.setPower(-.7);
                frontLeft.setPower(-.7);
                frontRight.setPower(-.7);
            }
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);


        }
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;


            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());



            if(x < 400) {
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

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();

                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setTargetPosition(-890);
                backRight.setTargetPosition(890);
                frontLeft.setTargetPosition(890);
                frontRight.setTargetPosition(-890);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setPower(-.3);
                backRight.setPower(.3);
                frontLeft.setPower(.3);
                frontRight.setPower(-.3);


                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

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
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                backLeft.setTargetPosition(-560);
                backRight.setTargetPosition(560);
                frontLeft.setTargetPosition(-560);
                frontRight.setTargetPosition(560);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setPower(-.5);
                backRight.setPower(.5);
                frontLeft.setPower(-.5);
                frontRight.setPower(.5);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

                backLeft.setPower(.3);
                backRight.setPower(.3);
                frontLeft.setPower(.3);
                frontRight.setPower(.3);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                sleep(1000);

                angleServo.setPosition(.2);
                sleep(1000);

                leftServo.setPosition(1);
                rightServo.setPosition(.5);

                sleep(2000);
                angleServo.setPosition(.4);


                backLeft.setTargetPosition(100);
                backRight.setTargetPosition(100);
                frontLeft.setTargetPosition(100);
                frontRight.setTargetPosition(100);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setPower(.2);
                backRight.setPower(.2);
                frontLeft.setPower(.2);
                frontRight.setPower(.2);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }
                angleServo.setPosition(.2);


                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



                backLeft.setTargetPosition(-250);
                backRight.setTargetPosition(-250);
                frontLeft.setTargetPosition(-250);
                frontRight.setTargetPosition(-250);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setPower(-.5);
                backRight.setPower(-.5);
                frontLeft.setPower(-.5);
                frontRight.setPower(-.5);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }



                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                backLeft.setTargetPosition(1330);
                backRight.setTargetPosition(-1330);
                frontLeft.setTargetPosition(1330);
                frontRight.setTargetPosition(-1330);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setPower(.2);
                backRight.setPower(-.2);
                frontLeft.setPower(.2);
                frontRight.setPower(-.2);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);


                backLeft.setTargetPosition(250);
                backRight.setTargetPosition(250);
                frontLeft.setTargetPosition(250);
                frontRight.setTargetPosition(250);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setPower(.3);
                backRight.setPower(.3);
                frontLeft.setPower(.3);
                frontRight.setPower(.3);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                backLeft.setTargetPosition(-280);
                backRight.setTargetPosition(280);
                frontLeft.setTargetPosition(280);
                frontRight.setTargetPosition(-280);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setPower(-.5);
                backRight.setPower(.5);
                frontLeft.setPower(.5);
                frontRight.setPower(-.5);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                sleep(1000);

                Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Slide.setTargetPosition(-1550);


                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slide.setPower(1);

                while (Slide.isBusy()) {
                    telemetry.addData("Slide at", Slide.getCurrentPosition());
                    telemetry.update();
                }

                rightServo.setPosition(0);

                sleep(2000);

                Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Slide.setTargetPosition(800);


                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slide.setPower(1);

                while (Slide.isBusy()) {
                    telemetry.addData("Slide at", Slide.getCurrentPosition());
                    telemetry.update();

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

                backLeft.setPower(-.5);
                backRight.setPower(.5);
                frontLeft.setPower(.5);
                frontRight.setPower(-.5);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

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

                backLeft.setPower(.3);
                backRight.setPower(.3);
                frontLeft.setPower(.3);
                frontRight.setPower(.3);

                while (frontRight.isBusy()) {
                    telemetry.addData("Encoders at", frontRight.getCurrentPosition());
                    telemetry.update();
                }

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);






            }else if (x > 800){
                //RIGHT


                backLeft.setTargetPosition(1200);
                backRight.setTargetPosition(1200);
                frontLeft.setTargetPosition(1200);
                frontRight.setTargetPosition(1200);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(.5);
                    backRight.setPower(.5);
                    frontLeft.setPower(.5);
                    frontRight.setPower(.5);
                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                backLeft.setTargetPosition(-700);
                backRight.setTargetPosition(700);
                frontLeft.setTargetPosition(700);
                frontRight.setTargetPosition(-700);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.7);
                    backRight.setPower(.7);
                    frontLeft.setPower(.7);
                    frontRight.setPower(-.7);
                }
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setTargetPosition(-900);
                backRight.setTargetPosition(900);
                frontLeft.setTargetPosition(-900);
                frontRight.setTargetPosition(900);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.7);
                    backRight.setPower(.7);
                    frontLeft.setPower(-.7);
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


                backLeft.setTargetPosition(-300);
                backRight.setTargetPosition(300);
                frontLeft.setTargetPosition(300);
                frontRight.setTargetPosition(-300);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.7);
                    backRight.setPower(.7);
                    frontLeft.setPower(.7);
                    frontRight.setPower(-.7);
                }

                backLeft.setTargetPosition(-500);
                backRight.setTargetPosition(-500);
                frontLeft.setTargetPosition(-500);
                frontRight.setTargetPosition(-500);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.7);
                    backRight.setPower(-.7);
                    frontLeft.setPower(-.7);
                    frontRight.setPower(-.7);
                }

                sleep(1000);




                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setTargetPosition(-1500);
                backRight.setTargetPosition(-1500);
                frontLeft.setTargetPosition(-1500);
                frontRight.setTargetPosition(-1500);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.5);
                    backRight.setPower(-.5);
                    frontLeft.setPower(-.5);
                    frontRight.setPower(-.5);
                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






            } else if (x>=400 && x<=800) {


                backLeft.setTargetPosition(1000);
                backRight.setTargetPosition(1000);
                frontLeft.setTargetPosition(1000);
                frontRight.setTargetPosition(1000);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(.5);
                    backRight.setPower(.5);
                    frontLeft.setPower(.5);
                    frontRight.setPower(.5);
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



                backLeft.setTargetPosition(-700);
                backRight.setTargetPosition(700);
                frontLeft.setTargetPosition(700);
                frontRight.setTargetPosition(-700);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.7);
                    backRight.setPower(.7);
                    frontLeft.setPower(.7);
                    frontRight.setPower(-.7);
                }
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setTargetPosition(-1000);
                backRight.setTargetPosition(1000);
                frontLeft.setTargetPosition(-1000);
                frontRight.setTargetPosition(1000);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.7);
                    backRight.setPower(.7);
                    frontLeft.setPower(-.7);
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

                backLeft.setTargetPosition(-1100);
                backRight.setTargetPosition(-1100);
                frontLeft.setTargetPosition(-1100);
                frontRight.setTargetPosition(-1100);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                backLeft.setTargetPosition(-300);
                backRight.setTargetPosition(300);
                frontLeft.setTargetPosition(300);
                frontRight.setTargetPosition(-300);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.7);
                    backRight.setPower(.7);
                    frontLeft.setPower(.7);
                    frontRight.setPower(-.7);
                }
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backLeft.setTargetPosition(-1500);
                backRight.setTargetPosition(-1500);
                frontLeft.setTargetPosition(-1500);
                frontRight.setTargetPosition(-1500);

                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (!(frontRight.getCurrentPosition() == frontRight.getTargetPosition())) {

                    backLeft.setPower(-.5);
                    backRight.setPower(-.5);
                    frontLeft.setPower(-.5);
                    frontRight.setPower(-.5);
                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




            }


        }   // end for() loop

    }   // end method telemetryTfod()




}
