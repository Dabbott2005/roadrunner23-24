package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

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

@Autonomous(name = "redRightRR")
public class redRightRR extends LinearOpMode {
    float getBrightnessRight() {
        NormalizedRGBA colors = colorSensorRight.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
        telemetry.update();

        return colors.alpha;
    }
    float getBrightnessLeft() {
        NormalizedRGBA colors = colorSensorLeft.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
        telemetry.update();

        return colors.alpha;
    }
    private NormalizedColorSensor colorSensorRight = null;
    private NormalizedColorSensor colorSensorLeft = null;

    static final double     WHITE_THRESHOLD = 0.5;  // spans between 0.0 - 1.0 from dark to light
    //TO IMPLEMENT SENSOR LOGIC: detecting white means WHITE_THRESHOLD>.5

    private static final double DOWN_ANGLE = 0.4;
    private static final double DEPO_ANGLE = 0.1;
    private static final double LEFT_OPEN = 1;
    private static final double RIGHT_OPEN = 0;

    //ext motors


    enum State {
        TRAJ_LEFT,   //moving forward while turning to left spike mark
        TRAJ_MIDDLE,   //
        TRAJ_RIGHT,

        TRAJ_TO_STACK,

        TRAJ_TO_BACKDROP,

        TRAJ_TO_PARK,

        IDLE,            // Our bot will enter the IDLE state when done
    }


    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(12, -61, (Math.toRadians(90)));

    OpenCvWebcam webcam1;

    @Override
    public void runOpMode() throws InterruptedException {
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

        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "Right Color");
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "Left Color");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        drive.setPoseEstimate(startPose);

        // If necessary, turn ON the white LED (if there is no LED switch on the sensor)
        if (colorSensorRight instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorRight).enableLight(true);
        }
        if (colorSensorLeft instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorLeft).enableLight(true);
        }

        // Some sensors allow you to set your light sensor gain for optimal sensitivity...
        // See the SensorColor sample in this folder for how to determine the optimal gain.
        // A gain of 15 causes a Rev Color Sensor V2 to produce an Alpha value of 1.0 at about 1.5" above the floor.
        colorSensorRight.setGain(15);
        colorSensorLeft.setGain(15);




        TrajectorySequence traj_right = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    lift.angleServo.setPosition(DEPO_ANGLE);
                    lift.liftLeft.setTargetPosition(3100);
                    lift.liftRight.setTargetPosition(3100);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(1);
                    lift.liftRight.setPower(1);
                })
                .lineToSplineHeading(new Pose2d(23,-42,(Math.toRadians(90))))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.leftServo.setPosition(LEFT_OPEN);
                })
                .waitSeconds(1)
                .back(6)
                .lineToSplineHeading(new Pose2d(42,-39,(Math.toRadians(0))))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(-1550);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.rightServo.setPosition(RIGHT_OPEN);
                })
                .waitSeconds(2)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(1540);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.angleServo.setPosition(DOWN_ANGLE);
                    lift.liftLeft.setTargetPosition(-3100);
                    lift.liftRight.setTargetPosition(-3100);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(.7);
                    lift.liftRight.setPower(.7);
                })
                .strafeLeft(15)
                .build();

        TrajectorySequence traj_middle = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    lift.angleServo.setPosition(DEPO_ANGLE);
                    lift.liftLeft.setTargetPosition(3100);
                    lift.liftRight.setTargetPosition(3100);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(1);
                    lift.liftRight.setPower(1);
                })
                .lineToSplineHeading(new Pose2d(10,-36,(Math.toRadians(90))))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.leftServo.setPosition(LEFT_OPEN);
                })
                .waitSeconds(2)
                .back(6)
                .lineToSplineHeading(new Pose2d(40,-36,(Math.toRadians(0))))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(-1550);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    lift.rightServo.setPosition(RIGHT_OPEN);
                })
                .waitSeconds(2)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(1540);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.angleServo.setPosition(DOWN_ANGLE);
                    lift.liftLeft.setTargetPosition(-3100);
                    lift.liftRight.setTargetPosition(-3100);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(.7);
                    lift.liftRight.setPower(.7);
                })
                .strafeLeft(15)
                .build();

        TrajectorySequence traj_left = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    lift.angleServo.setPosition(DEPO_ANGLE);
                    lift.liftLeft.setTargetPosition(3200);
                    lift.liftRight.setTargetPosition(3200);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(1);
                    lift.liftRight.setPower(1);
                })
                .forward(2)
                .strafeRight(2)
                .lineToSplineHeading(new Pose2d(7,-36,(Math.toRadians(180))))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.leftServo.setPosition(LEFT_OPEN);
                })
                .waitSeconds(2)
                .forward(4)
                .back(10)
                .lineToSplineHeading(new Pose2d(40,-29,(Math.toRadians(0))))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(-1550);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .waitSeconds(2)//
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    lift.rightServo.setPosition(RIGHT_OPEN);
                })
                .waitSeconds(2)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(1540);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.angleServo.setPosition(DOWN_ANGLE);
                    lift.liftLeft.setTargetPosition(-3100);
                    lift.liftRight.setTargetPosition(-3100);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(.7);
                    lift.liftRight.setPower(.7);
                })
                .strafeLeft(15)
                .build();

        TrajectorySequence cycle_pixels = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-50,50,(Math.toRadians(270)))) //move to the pixel stack
                .strafeLeft(5)
                .strafeRight(10)
                .strafeLeft(10)
                .strafeRight(10)
                .addTemporalMarker(0,() ->{
                    lift.Intake.setPower(-1);
                    if (getBrightnessLeft() > WHITE_THRESHOLD && getBrightnessRight() > WHITE_THRESHOLD) { //if pixel is detected by both sensors
                        lift.Intake.setPower(0);
                        currentState = State.TRAJ_TO_BACKDROP;
                        //pick up pixels
                    }
                })
                .build();
        TrajectorySequence score_backdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-40 , 50))
                .lineToSplineHeading(new Pose2d(35,50,(Math.toRadians(180)))) //move to the back stage
                .addTemporalMarker(0, () -> {
                    lift.angleServo.setPosition(DEPO_ANGLE);
                    lift.liftLeft.setTargetPosition(3000);
                    lift.liftRight.setTargetPosition(3000);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(1);
                    lift.liftRight.setPower(1);
                })
                .lineToSplineHeading(new Pose2d(40,-36,(Math.toRadians(0))))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(-1550);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    lift.rightServo.setPosition(RIGHT_OPEN);
                    lift.leftServo.setPosition(LEFT_OPEN);
                })
                .waitSeconds(2)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.angleServo.setPosition(DOWN_ANGLE);
                    lift.liftLeft.setTargetPosition(-3100);
                    lift.liftRight.setTargetPosition(-3100);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(.7);
                    lift.liftRight.setPower(.7);
                })
                .strafeLeft(15)

                        .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(50,10))
                        .build();

        waitForStart();


        // Save more CPU resources when camera is no longer needed.

        while (opModeIsActive()  && !isStopRequested()) {
            switch (detector.getLocation()) {
                case LEFT:
                    currentState = State.TRAJ_LEFT;
                    break;

                case MIDDLE:
                    currentState = State.TRAJ_MIDDLE;

                    break;
                case RIGHT:
                    currentState = State.TRAJ_RIGHT;

                    break;
            }
            //
            switch (currentState) {
                case TRAJ_LEFT:
                    drive.followTrajectorySequenceAsync(traj_left);
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (getRuntime() < 22 && (!drive.isBusy())) { //if we have time for another cycle and parking
                        currentState = State.TRAJ_TO_STACK;
                    } else {
                        currentState = State.TRAJ_TO_PARK;
                    }
                    break;
                case TRAJ_RIGHT:
                    drive.followTrajectorySequenceAsync(traj_right);
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state
                    if (getRuntime() < 22 && (!drive.isBusy())) { //if we have time for another cycle and parking
                        currentState = State.TRAJ_TO_STACK;
                    } else {
                        currentState = State.TRAJ_TO_PARK;

                    }
                case TRAJ_MIDDLE:
                    drive.followTrajectorySequenceAsync(traj_middle);
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (getRuntime() < 22 && (!drive.isBusy())) { //if we have time for another cycle and parking
                        currentState = State.TRAJ_TO_STACK;
                    } else {
                        currentState = State.TRAJ_TO_PARK;
                    }
                    break;
                case TRAJ_TO_BACKDROP:
                    drive.followTrajectorySequenceAsync(score_backdrop);

                    if (getRuntime() < 22 && (!drive.isBusy())) { //if we have time for another cycle and parking
                        currentState = State.TRAJ_TO_STACK;
                    } else {
                        currentState = State.TRAJ_TO_PARK;
                    }

                    break;
                case TRAJ_TO_STACK:
                    drive.followTrajectorySequenceAsync(cycle_pixels);

                    if (getRuntime() < 22 && (!drive.isBusy())) { //if we have time for another cycle and parking
                        currentState = State.TRAJ_TO_BACKDROP;
                    } else {
                        currentState = State.TRAJ_TO_PARK;
                    }
                    break;

                case TRAJ_TO_PARK:
                    drive.followTrajectorySequenceAsync(park);
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }

                    break;

                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            webcam1.stopStreaming();
            drive.update();

            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();


        }
    }// end runOpMode()
    public class Lift {
        private DcMotor Slide = null;

        private DcMotor Intake = null;

        private DcMotor liftLeft = null;

        private DcMotor liftRight = null;


        //Servos


        //servos
        private Servo leftServo = null;
        private Servo rightServo = null;
        private Servo angleServo = null;
        private Servo planeServo = null;

        private static final double KP = 0.04; // Placeholder value, adjust as needed
        private static final double KI = 0.0; // Placeholder value, adjust as needed
        private static final double KD = 0.0001; // Placeholder value, adjust as needed

        private double targetPosition;
        private double currentPosition;
        private double previousError;
        private double integral;

        private static final int SLIDE_UP = -3100;  // Example target position for linear slides
        private static final int SLIDE_DOWN = 3000;  // Example target position for linear slides
        private static final int ROTATE_UP = 1200;  // Example target position for grip rotation

        private static final int ROTATE_DOWN = -1200;  // Example target position for linear slides



        public Lift(HardwareMap hardwareMap) {
            Intake = hardwareMap.get(DcMotor.class, "Intake");
            Slide = hardwareMap.get(DcMotor.class, "Slide");
            liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
            liftRight = hardwareMap.get(DcMotor.class, "liftRight");


            leftServo = hardwareMap.get(Servo.class, "leftServo");
            rightServo = hardwareMap.get(Servo.class, "rightServo");
            angleServo = hardwareMap.get(Servo.class, "angleServo");
            planeServo = hardwareMap.get(Servo.class, "planeservo");
            Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            double error = targetPosition - currentPosition;

            // Calculate integral of the error
            integral += error;

            // Calculate derivative of the error
            double derivative = error - previousError;

            // Calculate output using PID formula
            double output = KP * error + KI * integral + KD * derivative;

            // Apply the output to the motor
            Slide.setPower(output);
            liftLeft.setPower(output);
            liftRight.setPower(output);

            // Update previous error for the next iteration
            previousError = error;
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
        public void setTargetPosition(double target) {
            targetPosition = target;
        }
        public void lowerLift() {
            // Implement logic to lower the lift to a predefined position
            liftLeft.setTargetPosition(SLIDE_DOWN);
            liftRight.setTargetPosition(SLIDE_DOWN);
        }

        public void raiseLift() {
            // Implement logic to raise the lift to a predefined position
            liftLeft.setTargetPosition(SLIDE_UP);
            liftRight.setTargetPosition(SLIDE_UP);
        }

        public void raiseSlide() {
            Slide.setTargetPosition(ROTATE_UP);
            // Implement logic to open the claw (if applicable)
            // This might involve controlling a servo or another motor
        }

        public void lowerSlide() {
            Slide.setTargetPosition(ROTATE_DOWN);
            // Implement logic to close the claw (if applicable)
            // This might involve controlling a servo or another motor
        }
    }




    /**
     * Initialize the TensorFlow Object Detection processor.
     */

}