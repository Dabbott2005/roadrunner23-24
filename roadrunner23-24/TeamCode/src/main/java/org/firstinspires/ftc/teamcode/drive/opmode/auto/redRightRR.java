package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.List;

@TeleOp(name = "redRightRR")
public class redRightRR extends LinearOpMode {

    enum State {
        TRAJ_LEFT,   //
        TRAJ_MIDDLE,   //
        TRAJ_RIGHT,         //
        TRAJ_BACKDROP_LEFT,//
        TRAJ_BACKDROP_MIDDLE,
        TRAJ_BACKDROP_RIGHT,

        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(11, -61, (Math.toRadians(90)));


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "redProp.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
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
        //Lift lift = new Lift(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj_left = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(11,-35,(Math.toRadians(180))))
                .addTemporalMarker(0, () -> {
                    //set GripRotate to DOWN
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(2,() -> {
                    //set GripRotate to UP
                })
                .build();

        TrajectorySequence traj_backdrop_left = drive.trajectorySequenceBuilder(traj_left.end())
                .addTemporalMarker(1.5,() -> {
                    //set LIFT to UP
                })
                .lineTo(new Vector2d(49, -32))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    //set depo to OPEN
                })
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    //set depo to close
                    //set LIFT to DOWN
                })
                .strafeLeft(25)

                .build();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            while (opModeIsActive()) {


                telemetryTfod();



                // Push telemetry to the Driver Station.
                telemetry.update();
                visionPortal.stopStreaming();
                // Save CPU resources; can resume streaming when needed.


                // Share the CPU.
            }
        }




        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
        while (opModeIsActive()  && !isStopRequested()) {
            switch (currentState) {
                case TRAJ_LEFT:
                    drive.followTrajectorySequenceAsync(traj_left);
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TRAJ_BACKDROP_LEFT;
                        drive.followTrajectorySequenceAsync(traj_backdrop_left);
                    }
                    break;
                //case TRAJ_RIGHT:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    //if (!drive.isBusy()) {
                        //currentState = State.TRAJ_BACKDROP_RIGHT;
                        //drive.turnAsync();
                    //}
                    //break;
                //case TURN_1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    //if (!drive.isBusy()) {
                        //currentState = State.TRAJECTORY_3;
                       // drive.followTrajectoryAsync(trajectory3);
                   // }
                   // break;
                //case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    //if (!drive.isBusy()) {
                        //currentState = State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        //waitTimer1.reset();
                    //}
                    //break;
                //case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    //if (waitTimer1.seconds() >= waitTime1) {
                        //currentState = State.TURN_2;
                        //drive.turnAsync(turnAngle2);
                    //}
                    //break;
                //case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    //if (!drive.isBusy()) {
                        //currentState = State.IDLE;
                    //}
                    //break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }


        }
    }


    // end runOpMode()

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
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

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
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

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
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {


        List<Recognition> currentRecognitions = tfod.getFreshRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;



            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x < 300) {
                telemetry.addData("Object Position", "Left");
                currentState = State.TRAJ_LEFT;

                // Perform actions for the object on the left.
                // Example: drive left or execute left-specific commands.
            } else if (x > 300) {
                telemetry.addData("Object Position", "Middle");


                // Perform actions for the object on the right.
                // Example: drive right or execute right-specific commands.
            } else if (tfod == null){
                telemetry.addData("Object Position", "Right");
                // Perform actions for the object in the middle.
                // Example: drive forward or execute middle-specific commands.
            }   // end for() loop

        }   // end method telemetryTfod()

    }
}