package org.firstinspires.ftc.teamcode.opmode.autonomous.vision.teamelement;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.imgproc.Imgproc;


import org.openftc.easyopencv.OpenCvPipeline;

class PropDetection extends LinearOpMode {
    OpenCvWebcam webcam = null;
    public void startup(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new PropDetectionPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1275, 639, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    class PropDetectionPipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat middleCrop;
        double leftAverageFinal;
        double rightAverageFinal;
        double middleAverageFinal;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(0,0,0);

        int position = 3;

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2BGR555);
            telemetry.addLine("looking lol");

            Rect leftRect = new Rect(1, 1, 425, 213);
            Rect middleRect = new Rect(425, 1, 425, 213);
            Rect rightRect = new Rect(850, 1, 425, 213);



            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            middleCrop = YCbCr.submat(middleRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(middleCrop, middleCrop, 2);

            Scalar leftAverage = Core.mean(leftCrop);
            Scalar rightAverage = Core.mean(rightCrop);
            Scalar middleAverage = Core.mean(middleCrop);

            leftAverageFinal = leftAverage.val[0];
            rightAverageFinal = rightAverage.val[0];
            middleAverageFinal = middleAverage.val[0];

            if (leftAverageFinal > rightAverageFinal || leftAverageFinal > middleAverageFinal){
                position = 1;
                telemetry.addLine("left");
            }
            if (middleAverageFinal > rightAverageFinal || leftAverageFinal < middleAverageFinal){
                position = 2;
                telemetry.addLine("middle");
            }
            else {
                position = 3;
                telemetry.addLine("right");
            }
            return (output);

    }



}}