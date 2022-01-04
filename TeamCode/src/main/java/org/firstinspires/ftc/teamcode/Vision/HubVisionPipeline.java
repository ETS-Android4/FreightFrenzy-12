package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Config
@Autonomous
public class HubVisionPipeline extends LinearOpMode {

    private static int rows = 640;
    private static int cols = 480;
    public static int sampleWidth = 20;
    public static int sampleHeight = 10;
    public static int satThresh = 210, redThresh = 25, blockSize = 35, C = 0, structuringElementSize = 3;
    public int duckLocation = -1;
    public int colorSpace = 41;
    public static int rotateAngle = 90;
    public static int extract = 1;
    public static int g;
    public static int exp;

    OpenCvWebcam webCam, webcam2;

    @Override
    public void runOpMode() throws InterruptedException {

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), viewportContainerIds[0]);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"), viewportContainerIds[1]);
        webCam.openCameraDevice();//open camera
        webcam2.openCameraDevice();//open camera
        webCam.setPipeline(new lowerCameraPipeline());//different stages
        webcam2.setPipeline(new upperCameraPipeline());//different stages
         */

        //1 camera at the moment.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        webCam.openCameraDevice();//open camera
        webCam.setPipeline(new duckScanPipeline());
        webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        FtcDashboard.getInstance().startCameraStream(webCam, 0);

        ExposureControl exposure = webCam.getExposureControl();
        GainControl gain = webCam.getGainControl();
        exposure.setMode(ExposureControl.Mode.Manual);
        g = gain.getGain();
        exp = (int) exposure.getExposure(TimeUnit.MILLISECONDS);

        //webcam2.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        //all of our movement jazz
        while (opModeIsActive()) {
            gain.setGain(g);
            exposure.setExposure(exp, TimeUnit.MILLISECONDS);
            telemetry.addData("Current Exposure", exp);
            telemetry.addData("Current Gain", g);
            telemetry.addData("Max Gain", gain.getMaxGain());
            telemetry.addData("Min Gain", gain.getMinGain());
            telemetry.addData("Max Exposure", exposure.getMaxExposure(TimeUnit.MILLISECONDS));
            telemetry.addData("Min Exposure", exposure.getMinExposure(TimeUnit.MILLISECONDS));

            telemetry.update();
            sleep(100);
        }
    }

    //detection pipeline
    public static class duckScanPipeline extends OpenCvPipeline
    {
        Mat rawMat = new Mat();
        Mat redMat = new Mat();
        Mat saturationMat = new Mat();
        Mat finalMat = new Mat();

        enum Stage
        {
            RAW,
            EXTRACT,
            RED
        }

        private Stage stageToRenderToViewport = Stage.EXTRACT;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            rawMat = input;

            Imgproc.cvtColor(rawMat, saturationMat, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(saturationMat, saturationMat, 1);
            Core.extractChannel(rawMat, redMat, 0);
            //Imgproc.threshold(saturationMat, finalMat, satThresh, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(redMat, redMat, redThresh, 255, Imgproc.THRESH_TOZERO_INV);
            Imgproc.adaptiveThreshold(redMat, finalMat, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, blockSize, C);
            Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(structuringElementSize, structuringElementSize));
            Imgproc.morphologyEx(finalMat, finalMat, Imgproc.MORPH_OPEN, element);
            Mat element1 = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(35, 35));
            Imgproc.morphologyEx(finalMat, finalMat, Imgproc.MORPH_DILATE, element1);
            double leftEdge = -1, rightEdge = -1;
            for(int x = 0; x < finalMat.cols(); x++){
                double[] pixel = finalMat.get(450, x);
                if(pixel[0] == 255){
                    leftEdge = pixel[0];
                    break;
                }
            }
            for(int x = finalMat.cols()-1; x>0; x--){
                double[] pixel = finalMat.get(450,x);
                if(pixel[0] == 255){
                    rightEdge = pixel[0];
                    break;
                }
            }
            Imgproc.cvtColor(finalMat, finalMat, Imgproc.COLOR_GRAY2RGB);
            Imgproc.line(finalMat, new Point(1, 450), new Point(639, 450), new Scalar(0,255,0), 3);
            Imgproc.line(finalMat, new Point(leftEdge, 0), new Point(rightEdge, 479), new Scalar(255,0,0), 7);
            switch (stageToRenderToViewport){
                case RAW:
                {
                    return rawMat;
                }
                case EXTRACT:
                {
                    return finalMat;
                }
                default:
                {
                    return input;
                }
            }
        }

    }

    static class upperCameraPipeline extends OpenCvPipeline
    {

        Mat inputMat = new Mat();
        Mat grayMat = new Mat();
        Mat interMat = new Mat();
        Mat outputMat = new Mat();
        Mat hierarchy = new Mat();

        enum Stage
        {
            INPUT,
            INTER,
            OUTPUT
        }

        private Stage stageToRenderToViewport = Stage.INTER;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputMat = input;
            //Mat rota = Imgproc.getRotationMatrix2D(new Point(160, 120), rotateAngle,1);
            //Imgproc.warpAffine(inputMat, inputMat, rota, new Size(320,240));
            Imgproc.cvtColor(inputMat, interMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(interMat, interMat, extract);
            Imgproc.medianBlur(interMat, interMat, 5);
            grayMat = interMat.clone();
            Imgproc.threshold(interMat, interMat, redThresh, 255, Imgproc.THRESH_BINARY);
            Imgproc.cvtColor(interMat, outputMat, Imgproc.COLOR_GRAY2RGB);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(interMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            Point[] centers = new Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            for (int i = 0; i < contours.size(); i++) {
                Imgproc.drawContours(outputMat, contoursPolyList, i, new Scalar(0,255,0), 3);
                Imgproc.rectangle(outputMat, boundRect[i].tl(), boundRect[i].br(), new Scalar(255,0,0), 2);
                Imgproc.line(outputMat, new Point((boundRect[contours.size()-1].tl().x+boundRect[contours.size()-1].br().x)/2, 0), new Point((boundRect[contours.size()-1].tl().x+boundRect[contours.size()-1].br().x)/2, 480), new Scalar(0,0,255), 3);
                //Imgproc.putText(outputMat, "Points: " + contoursPoly[i].rows(), boundRect[i].tl(), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
            }

            switch (stageToRenderToViewport){
                case INPUT:
                {
                    return inputMat;
                }
                case INTER:
                {
                    return outputMat;
                }
                default:
                {
                    return input;
                }
            }
        }

    }
}
