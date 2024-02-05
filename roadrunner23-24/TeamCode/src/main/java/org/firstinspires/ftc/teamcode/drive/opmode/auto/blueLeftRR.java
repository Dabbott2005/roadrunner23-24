package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "BlueLeftRR")
public class blueLeftRR extends LinearOpMode {
    private static final double DOWN_ANGLE = 0.4;
    private static final double DEPO_ANGLE = 0.1;
    private static final double LEFT_OPEN = 1;
    private static final double RIGHT_OPEN = 0;

    //ext motors


    enum State {
        TRAJ_LEFT,   //moving forward while turning to left spike mark
        TRAJ_MIDDLE,   //
        TRAJ_RIGHT,         //

        IDLE,            // Our bot will enter the IDLE state when done
    }


    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(12, 61, (Math.toRadians(270)));

    OpenCvWebcam webcam1;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        BluePropDectector detector = new BluePropDectector(telemetry);
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj_right = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    lift.angleServo.setPosition(DEPO_ANGLE);
                    lift.liftLeft.getCurrentPosition();
                    lift.liftRight.getCurrentPosition();
                    lift.liftLeft.setTargetPosition(3100);
                    lift.liftRight.setTargetPosition(3100);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(1);
                    lift.liftRight.setPower(1);
                })
                .lineToSplineHeading(new Pose2d(9,37,(Math.toRadians(180))))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.leftServo.setPosition(LEFT_OPEN);
                })
                .waitSeconds(2)
                .back(6)
                .lineToSplineHeading(new Pose2d(41,32,(Math.toRadians(0))))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(-1550);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    lift.rightServo.setPosition(RIGHT_OPEN);
                })
                .waitSeconds(1)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(800);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .strafeLeft(27)
                .forward(10)
                .waitSeconds(20)
                .build();

        TrajectorySequence traj_middle = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    lift.angleServo.setPosition(DEPO_ANGLE);
                    lift.liftLeft.getCurrentPosition();
                    lift.liftRight.getCurrentPosition();
                    lift.liftLeft.setTargetPosition(3200);
                    lift.liftRight.setTargetPosition(3200);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(1);
                    lift.liftRight.setPower(1);
                })
                .lineToSplineHeading(new Pose2d(13,35,(Math.toRadians(270))))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.leftServo.setPosition(LEFT_OPEN);
                })
                .waitSeconds(1)
                .forward(3)
                .back(9)
                .lineToSplineHeading(new Pose2d(43,39,(Math.toRadians(0))))
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
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    lift.Slide.getCurrentPosition();
                    lift.Slide.setTargetPosition(800);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .strafeLeft(26)
                .forward(12)
                .waitSeconds(20)
                .build();

        TrajectorySequence traj_left = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    lift.angleServo.setPosition(DEPO_ANGLE);
                    lift.liftLeft.getCurrentPosition();
                    lift.liftRight.getCurrentPosition();
                    lift.liftLeft.setTargetPosition(3100);
                    lift.liftRight.setTargetPosition(3100);
                    lift.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.liftLeft.setPower(1);
                    lift.liftRight.setPower(1);
                })
                .lineToSplineHeading(new Pose2d(23,40,(Math.toRadians(270))))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.leftServo.setPosition(LEFT_OPEN);
                })
                .waitSeconds(1)
                .forward(3)
                .back(9)
                .lineToSplineHeading(new Pose2d(42.5,43,(Math.toRadians(0))))
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
                    lift.Slide.setTargetPosition(800);
                    lift.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.Slide.setPower(1);
                })
                .strafeLeft(19)
                .forward(15)
                .waitSeconds(20)
                .build();

        waitForStart();





        // Save more CPU resources when camera is no longer needed.

        while (opModeIsActive()  && !isStopRequested()) {
            switch (detector.getLocation()) {
                case LEFT:
                    currentState = State.TRAJ_LEFT;
                    drive.followTrajectorySequenceAsync(traj_left);
                    break;

                case MIDDLE:
                    currentState = State.TRAJ_MIDDLE;
                    drive.followTrajectorySequenceAsync(traj_middle);

                    break;
                case RIGHT:
                    currentState = State.TRAJ_RIGHT;
                    drive.followTrajectorySequenceAsync(traj_right);

                    break;
            }
            switch (currentState) {
                case TRAJ_LEFT:
                    drive.followTrajectorySequence(traj_left);
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case TRAJ_RIGHT:
                    drive.followTrajectorySequence(traj_right);
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case TRAJ_MIDDLE:
                    drive.followTrajectorySequence(traj_middle);
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
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