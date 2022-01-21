package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import java.util.Comparator;
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
@TeleOp
public class HubVisionPipeline extends LinearOpMode {

    public final int rows = 640;
    public final int cols = 480;
    public static int hueMin = 115, hueMax = 125, satMin = 130, satMax = 210, valMin = 100, valMax = 150;
    public static int blurSize = 11, erodeSize = 15, dilateSize = 25;
    public static int extract = 1;
    public static int g;
    public static int exp;

    public static Stage stageToRenderToViewport = Stage.RAW;
    public static Stage[] stages = Stage.values();

    public static int currentStageNum = stageToRenderToViewport.ordinal();
    public static int nextStageNum = currentStageNum + 1;

    private static Point centerPointHub = new Point(320, 240);
    
    OpenCvWebcam webCam, webcam2;

    public static enum Stage
    {
        RAW,
        BLUR,
        HSVTHRESH,
        MORPH,
        FINAL
    }

    public static Point getCenterPointHub() {
        return centerPointHub;
    }

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
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //1 camera at the moment.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        webCam.openCameraDevice();//open camera
        webCam.setPipeline(new hubScanPipeline());
        webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        FtcDashboard.getInstance().startCameraStream(webCam, 0);

        ExposureControl exposure = webCam.getExposureControl();
        GainControl gain = webCam.getGainControl();
        exposure.setMode(ExposureControl.Mode.Manual);
        g = gain.getGain();
        exp = (int) exposure.getExposure(TimeUnit.MILLISECONDS);
        exp = 14;
        g = 0;

        //webcam2.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        //all of our movement jazz
        while (opModeIsActive()) {
            gain.setGain(g);
            exposure.setExposure(exp, TimeUnit.MILLISECONDS);
            TelemetryPacket cameraSettingsPacket = new TelemetryPacket();
            cameraSettingsPacket.put("Current Exposure", exp);
            cameraSettingsPacket.put("Current Gain", g);
            cameraSettingsPacket.put("Max Gain", gain.getMaxGain());
            cameraSettingsPacket.put("Min Gain", gain.getMinGain());
            cameraSettingsPacket.put("Max Exposure", exposure.getMaxExposure(TimeUnit.MILLISECONDS));
            cameraSettingsPacket.put("Min Exposure", exposure.getMinExposure(TimeUnit.MILLISECONDS));
            dashboard.sendTelemetryPacket(cameraSettingsPacket);
            sleep(100);
        }
    }

    //detection pipeline
    public static class hubScanPipeline extends OpenCvPipeline
    {
        Mat rawMat = new Mat();
        Mat blurredMat = new Mat();
        Mat hsvMat = new Mat();
        Mat mask = new Mat();
        Mat morphedMat = new Mat();
        Mat finalMat = new Mat();

        enum Stage
        {
            RAW,
            BLUR,
            HSVTHRESH,
            MORPH,
            FINAL
        }

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
            if(hueMin < 0) hueMin = 0;
            if(hueMax > 180) hueMax = 180;
            if(satMin < 0) satMin = 0;
            if(satMax > 255) satMax = 255;
            if(valMin < 0) valMin = 0;
            if(valMax > 255) valMax = 255;

            Imgproc.blur(rawMat, blurredMat, new Size(blurSize, blurSize));
            Imgproc.cvtColor(blurredMat, hsvMat, Imgproc.COLOR_BGR2HSV);

            Scalar minValues = new Scalar(hueMin, satMin, valMin);
            Scalar maxValues = new Scalar(hueMax, satMax, valMax);

            Core.inRange(hsvMat, minValues, maxValues, mask);

            Mat erode = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(erodeSize, erodeSize));
            Mat dilate = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(dilateSize, dilateSize));

            Imgproc.erode(mask, morphedMat, erode);
            Imgproc.dilate(morphedMat, morphedMat, dilate);

            morphedMat.copyTo(finalMat);
            Imgproc.cvtColor(finalMat, finalMat, Imgproc.COLOR_GRAY2BGR);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(morphedMat, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
            if(hierarchy.size().height > 0 && hierarchy.size().width >0){
                contours.sort(new Comparator<MatOfPoint>() {
                    @Override
                    public int compare(MatOfPoint c1, MatOfPoint c2) {
                        return (int) (Imgproc.contourArea(c1) - Imgproc.contourArea(c2));
                    }
                });
                Imgproc.drawContours(finalMat, contours,contours.size()-1, new Scalar(0, 255, 0), 5);
                //Bounding box of largest contour
                Rect rect = Imgproc.boundingRect(contours.get(contours.size()-1));
                //Center of bounding box
                centerPointHub = new Point((rect.tl().x+rect.br().x)*0.5, (rect.tl().y+rect.br().y)*0.5);
                Imgproc.circle(finalMat, centerPointHub, 5, new Scalar(0, 0, 255), 7);
                //Draw bounding box
                Imgproc.rectangle(finalMat, rect, new Scalar(255, 0, 0));

                //Draw center of mass of largest contour
                //Scalar centerOfMass = Core.mean(contours.get(contours.size()-1));
                //Imgproc.circle(finalMat, new Point(centerOfMass.val[0], centerOfMass.val[1]), 5, new Scalar(255, 0, 0), 7);
            }
            else centerPointHub = new Point(320, 240);


//            double leftEdge = -1, rightEdge = -1;
//            for(int x = 0; x < finalMat.cols(); x++){
//                double[] pixel = finalMat.get(450, x);
//                if(pixel[0] == 255){
//                    leftEdge = pixel[0];
//                    break;
//                }
//            }
//            for(int x = finalMat.cols()-1; x>0; x--){
//                double[] pixel = finalMat.get(450,x);
//                if(pixel[0] == 255){
//                    rightEdge = pixel[0];
//                    break;
//                }
//            }
//            Imgproc.cvtColor(finalMat, finalMat, Imgproc.COLOR_GRAY2RGB);
//            Imgproc.line(finalMat, new Point(1, 450), new Point(639, 450), new Scalar(0,255,0), 3);
//            Imgproc.line(finalMat, new Point(leftEdge, 0), new Point(rightEdge, 479), new Scalar(255,0,0), 7);
            switch (stageToRenderToViewport) {
                case RAW: {
                    return rawMat;
                }
                case BLUR:
                {
                    return hsvMat;
                }
                case HSVTHRESH:
                {
                    return mask;
                }
                case MORPH:
                {
                    return morphedMat;
                }
                case FINAL:
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
}
