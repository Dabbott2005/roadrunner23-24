package org.firstinspires.ftc.teamcode.drive.opmode.auto;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
public class TrajectoryOnly extends LinearOpMode {

    private static final double CLOSED_CLAW = 1;
    private static final double OPEN_CLAW = 0;
    private static final double OPEN_DEPO = 1;
    private static final double CLOSED_DEPO = 0.5;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        LEFT_TRAJECTORY_1,
        LEFT_TRAJECTORY_2,
        LEFT_WAIT_1,
        LEFT_WAIT_2,
        LEFT_TRAJECTORY_BACKDROP_1,
        LEFT_WAIT_3,
        LEFT_TRAJECTORY_BACKDROP_2,
        IDLE
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(15, 10, Math.toRadians(180));

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

    @Override
    public void runOpMode() throws InterruptedException {

        initTfod();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectoryOnly.Lift lift = new TrajectoryOnly.Lift(hardwareMap);
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory leftTrajectory1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(11,-35,(Math.toRadians(180))))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory leftTrajectory2 = drive.trajectoryBuilder(leftTrajectory1.end())
                .back(2)
                .build();

        Trajectory leftTrajectoryBackdrop1 = drive.trajectoryBuilder(leftTrajectory2.end())
                .lineTo(new Vector2d(49, -32))
                .build();

        Trajectory leftTrajectoryBackdrop2 = drive.trajectoryBuilder(leftTrajectoryBackdrop1.end())
                .forward(3)
                .build();

        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));


        // Define a 1.5 second wait time
        double waitTime1 = 2;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.LEFT_TRAJECTORY_1;
        drive.followTrajectoryAsync(leftTrajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case LEFT_TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.LEFT_TRAJECTORY_2;
                        drive.followTrajectoryAsync(leftTrajectory2);
                    }
                    break;
                case LEFT_TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, waiting, once finished
                    if (!drive.isBusy()) {
                        lift.claw.setPosition(CLOSED_CLAW);
                        lift.setTargetPosition(redRightRR.Lift.ROTATE_DOWN);
                        currentState = State.LEFT_WAIT_1;
                        waitTimer1.reset();
                    }
                    break;
                case LEFT_WAIT_1:
                    // Check if wait time is over
                    if (waitTimer1.seconds() >= waitTime1) {
                        lift.claw.setPosition(OPEN_CLAW);
                        currentState = State.LEFT_WAIT_2;

                        waitTimer1.reset();
                    }
                    break;
                case LEFT_WAIT_2:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (waitTimer1.seconds() >= waitTime1) {
                        lift.claw.setPosition(CLOSED_CLAW);
                        lift.setTargetPosition(redRightRR.Lift.ROTATE_UP);
                        lift.setTargetPosition(redRightRR.Lift.SLIDE_UP);
                        currentState = State.LEFT_TRAJECTORY_BACKDROP_1;
                        drive.followTrajectoryAsync(leftTrajectoryBackdrop1);

                        waitTimer1.reset();
                    }
                    break;
                case LEFT_TRAJECTORY_BACKDROP_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {

                        currentState = State.LEFT_WAIT_3;
                    }
                    break;
                case LEFT_WAIT_3:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (waitTimer1.seconds() >= waitTime1) {
                        lift.deposit.setPosition(OPEN_DEPO);
                        currentState = State.LEFT_TRAJECTORY_BACKDROP_2;
                        drive.followTrajectoryAsync(leftTrajectoryBackdrop2);
                        waitTimer1.reset();
                    }
                    break;
                case LEFT_TRAJECTORY_BACKDROP_2:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {

                        currentState = State.LEFT_WAIT_3;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }



    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    public class Lift {
        private DcMotorEx leftSlide;
        private DcMotorEx rightSlide;
        private DcMotorEx gripRotate;

        //servos
        public Servo claw = null;
        public Servo deposit = null;

        private static final double KP = 0.004; // Placeholder value, adjust as needed
        private static final double KI = 0.0; // Placeholder value, adjust as needed
        private static final double KD = 0.0001; // Placeholder value, adjust as needed

        private double targetPosition;
        private double currentPosition;
        private double previousError;
        private double integral;

        private static final int SLIDE_UP = 1100;  // Example target position for linear slides
        private static final int SLIDE_DOWN = 0;  // Example target position for linear slides
        private static final int ROTATE_UP = 0;  // Example target position for grip rotation

        private static final int ROTATE_DOWN = 400;  // Example target position for linear slides



        public Lift(HardwareMap hardwareMap) {
            leftSlide = hardwareMap.get(DcMotorEx.class, "leftslide");
            rightSlide = hardwareMap.get(DcMotorEx.class, "rightslide");
            gripRotate = hardwareMap.get(DcMotorEx.class, "griprotation");

            claw = hardwareMap.get(Servo.class, "clawservo");
            deposit = hardwareMap.get(Servo.class, "depositservo");

            leftSlide.setDirection(DcMotor.Direction.FORWARD);
            rightSlide.setDirection(DcMotor.Direction.FORWARD);
            gripRotate.setDirection(DcMotor.Direction.FORWARD);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gripRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set zero power behavior
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            gripRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            rightSlide.setPower(output);
            leftSlide.setPower(output);
            gripRotate.setPower(output);

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
            leftSlide.setTargetPosition(SLIDE_DOWN);
            rightSlide.setTargetPosition(SLIDE_DOWN);
        }

        public void raiseLift() {
            // Implement logic to raise the lift to a predefined position
            leftSlide.setTargetPosition(SLIDE_UP);
            rightSlide.setTargetPosition(SLIDE_UP);
        }

        public void raiseClaw() {
            gripRotate.setTargetPosition(ROTATE_UP);
            // Implement logic to open the claw (if applicable)
            // This might involve controlling a servo or another motor
        }

        public void lowerClaw() {
            gripRotate.setTargetPosition(ROTATE_DOWN);
            // Implement logic to close the claw (if applicable)
            // This might involve controlling a servo or another motor
        }
    }
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
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {


        List<Recognition> currentRecognitions = tfod.getFreshRecognitions();

        telemetry.addData("# Objects Detected", currentRecognitions.size());
        if (currentRecognitions.size() > 0) {



            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;


                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                if (x < 400) {
                    telemetry.addData("Object Position", "Left");
                    currentState = TrajectoryOnly.State.TRAJ_LEFT;

                    // Perform actions for the object on the left.
                    // Example: drive left or execute left-specific commands.
                } else if (x > 400) {
                    telemetry.addData("Object Position", "Middle");
                    currentState = TrajectoryOnly.State.TRAJ_MIDDLE;

                }

            }// end for() loop

        }else{
            telemetry.addData("No objects detected","defaulting to Right");
            currentState = TrajectoryOnly.State.TRAJ_RIGHT;

        }

    }
}