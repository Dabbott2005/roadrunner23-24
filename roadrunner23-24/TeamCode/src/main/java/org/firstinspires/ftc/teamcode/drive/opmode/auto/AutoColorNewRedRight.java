package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoColorNewRedRight extends OpMode {
    OpenCvWebcam webcam1 = null;
    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new redPipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    @Override
    public void loop() {

    }

}
class exaplePipeline extends OpenCvPipeline {
    Mat YCBCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    double leftavgfin;
    double rightavgfin;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(225.0, 0.0, 0.0);



     public Mat processFrame(Mat input){
         Mat YCbCr;
         Imgproc.cvtColor(input,YCbCr,Imgproc.COLOP_RGB2YCrCb);
         telemetry.addLine("pipeline running");


         Rect leftRect = new Rect(1,)

     }
}
