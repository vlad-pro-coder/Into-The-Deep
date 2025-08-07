package org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Logitech920 {

    OpenCvWebcam webcam;
    Pipeline pipeline;
    public YellowSampleDetectionPipeline yellow;
    private class Pipeline extends OpenCvPipeline{
        public Mat lastFrame = new Mat(); // Store last frame

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(lastFrame); // Save current frame
            return input; // Show it on screen (optional)
        }
    }

    public Logitech920(String name, HardwareMap hardwaremap){
        int cameraMonitorViewId = hardwaremap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwaremap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwaremap.get(WebcamName.class, name), cameraMonitorViewId);

        pipeline = new Pipeline();
        yellow = new YellowSampleDetectionPipeline();
        webcam.setPipeline(yellow);
    }

    public void startCamera() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                // ------------------ Tzeapa frate
            }

        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }


}
