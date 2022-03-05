package org.firstinspires.ftc.teamcode.Autonomous.RRPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.BlueQualTeleOp;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.colorSpace;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.extract;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.sampleHeight;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.sampleWidth;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.thresh;

import java.util.Arrays;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */

@Config
@Autonomous(group = "drive")
public class FreightRedPathWarehouse extends LinearOpMode {

    public static double back = 58;

    public static double pow = -1;

    private Pose2d startPose = new Pose2d(-60, 0, Math.toRadians(0)); //Need to vary heading

    public static int middleX = 100, rightX = 240, allY = 195, leftX = 30;

    public static int x1 = -40, x2 = -16;

    public Pose2d pose1 = new Pose2d(x1, -3), score = new Pose2d(-74, -4), warehouse = new Pose2d(x2, -1);

    public static int duckLocation = -1, manualDuckLocation = -1;

    public static double level0 = 375, level1 = 1450, level2 = 1650, sensorSideOffset, sensorStrightOffset;

    public static double OPEN = 0.02, CLOSED = 0.65, HALF = 0.21;

    public MinVelocityConstraint slowVel = new MinVelocityConstraint(
            Arrays.asList(new AngularVelocityConstraint(Math.toRadians(360)), new MecanumVelocityConstraint(15,
                    DriveConstants.TRACK_WIDTH)));
    public ProfileAccelerationConstraint slowAccel = new ProfileAccelerationConstraint(10);

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        webCam.openCameraDevice();//open camera
        webCam.setPipeline(new duckScanPipeline());
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//display on RC
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        double lastTime = 0;

        drive.leftIntakeLift.setTargetPosition(BlueQualTeleOp.intakeLiftLevels[2] + 100);
        drive.leftIntakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftIntakeLift.setPower(0.4);
        drive.rightIntakeLift.setTargetPosition(BlueQualTeleOp.intakeLiftLevels[2] + 100);
        drive.rightIntakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightIntakeLift.setPower(0.4);

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Duck Location: ", duckLocation);
            telemetry.update();
        }

        ElapsedTime time;
        waitForStart();
        if(isStopRequested()) return;

        time = new ElapsedTime();

        double slideTicks = duckLocation == 0 ? level0 : duckLocation == 1 ? level1 : level2; //0;
//        if(duckLocation > 0) slideTicks = duckLocation == 1 ? level1 : level2;
//        if(manualDuckLocation != -1) slideTicks = manualDuckLocation < 2 ? (manualDuckLocation == 0 ? 0 : level1) : level2;

        System.out.println("Duck: " + duckLocation + ", ticks: " + slideTicks);

        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.update();

        drive.leftIntakeLift.setTargetPosition(BlueQualTeleOp.intakeLiftLevels[1]);
        drive.rightIntakeLift.setTargetPosition(BlueQualTeleOp.intakeLiftLevels[1]);

        sleep(400);

        drive.slides.setTargetPosition((int) slideTicks);
        drive.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.slides.setPower(1);
        if(slideTicks == level2) drive.arm.setPosition(BlueQualTeleOp.ARMFRONT);
        else if(slideTicks == level1) drive.arm.setPosition(BlueQualTeleOp.ARMMID);
        else drive.arm.setPosition(BlueQualTeleOp.ARMMID); //drive.arm.setPosition(BlueQualTeleOp.ARMBOT);

        Trajectory offWall = drive.trajectoryBuilder(startPose)
                .back(18)
                .build();
        drive.followTrajectory(offWall);

        if(slideTicks == level0) {
            Trajectory toHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(16)
                    .build();
            drive.followTrajectory(toHub);

            drive.lid.setPosition(BlueQualTeleOp.DUMPMID);
            sleep(650);

            Trajectory towall = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight(17)
                    .build();
            drive.followTrajectory(towall);
        }

        if(slideTicks == level2) drive.lid.setPosition(BlueQualTeleOp.DUMP);
        else if(slideTicks == level1) drive.lid.setPosition(BlueQualTeleOp.DUMPMID);
        sleep(500);
        drive.lid.setPosition(BlueQualTeleOp.CLOSE);
        drive.arm.setPosition(BlueQualTeleOp.ARMBACK);
        drive.slides.setTargetPosition(BlueQualTeleOp.levels[0]);

        while(time.seconds() < 20 && !isStopRequested()) {

            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.convertSensorInput(drive.back.getVoltage())));

            drive.slides.setTargetPosition(BlueQualTeleOp.levels[0]);

            Trajectory backup = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeTo(pose1.vec())
                    .build();
            drive.followTrajectory(backup);

            drive.rightIntakeLift.setTargetPosition(BlueQualTeleOp.intakeLiftLevels[0]);

            drive.rightIntakeSpinner.setPower(1);

            drive.lid.setPosition(BlueQualTeleOp.OPEN);

            Trajectory pickup = drive.trajectoryBuilder(drive.getPoseEstimate(), false, slowVel, slowAccel)
                    .strafeTo(warehouse.vec())
                    .build();
            drive.followTrajectoryAsync(pickup);

            while (!isStopRequested() && drive.isBusy() && drive.rightLimit.getState()) {
                drive.update();
            }

            drive.setPoseEstimate(pickup.end());

            drive.update();

            drive.setMotorPowers(0, 0, 0, 0);

            sleep(400);

            drive.rightIntakeLift.setTargetPosition(BlueQualTeleOp.intakeLiftLevels[2]);

            sleep(650);
            drive.rightIntakeSpinner.setPower(1);
            sleep(200);
            drive.rightIntakeSpinner.setPower(-1);
            sleep(200);
            drive.rightIntakeSpinner.setPower(0);
            drive.rightIntakeLift.setTargetPosition(BlueQualTeleOp.intakeLiftLevels[1] + 100);
            drive.rightIntakeLift.setPower(0.4);
            sleep(200);
            drive.lid.setPosition(BlueQualTeleOp.CLOSE);
            drive.slides.setTargetPosition((int) level2);

            System.out.println("Sensors: " + drive.convertSensorInput(drive.right.getVoltage()) + ", " + drive.convertSensorInput(drive.back.getVoltage()));

            drive.setPoseEstimate(new Pose2d(2 - drive.convertSensorInput(drive.right.getVoltage()), drive.convertSensorInput(drive.back.getVoltage())));

            Trajectory toHub = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                    .splineToConstantHeading(pose1.vec(), Math.toRadians(180))
                    .splineToConstantHeading(score.vec(), Math.toRadians(180))
                    .build();
            drive.followTrajectory(toHub);

            drive.arm.setPosition(BlueQualTeleOp.ARMFRONT);
            sleep(400);
            drive.lid.setPosition(BlueQualTeleOp.DUMP);
            sleep(650);
            drive.lid.setPosition(BlueQualTeleOp.CLOSE);
            sleep(200);
            drive.arm.setPosition(BlueQualTeleOp.ARMBACK);
            sleep(300);
        }

        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.convertSensorInput(drive.back.getVoltage())));

        drive.slides.setTargetPosition(BlueQualTeleOp.levels[0]);

        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(warehouse.getX() + 4, warehouse.getY()), Math.toRadians(0))
                .build();
        drive.followTrajectory(park);
    }

    public void imuTurn(double angle) {
        //Radians
        double imuHeading = drive.imu.getAngularOrientation().firstAngle;
        while((Math.abs(imuHeading - angle)>Math.toRadians(2)) && !isStopRequested() && opModeIsActive()){
            System.out.println("Heading check: " + Math.abs(imuHeading - angle));
            System.out.println("Angle: " + angle);
            drive.update();
            imuHeading = drive.imu.getAngularOrientation().firstAngle;
            double tempHeading = imuHeading;
            double tempTarget = angle;
            System.out.println("Heading: " + imuHeading);
            if(tempHeading < 0) tempHeading += 2 * Math.PI;
            if(angle < 0) tempTarget += 2 * Math.PI;
            double p = 0.3, f = 0.04;
            //int invert = tempHeading + (2 * Math.PI - tempTarget) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double invert = angle - imuHeading;
            if(invert > Math.PI) invert -= 2 * Math.PI;
            else if(invert < -Math.PI) invert += 2 * Math.PI;
            invert = invert < 0 ? 1 : -1;
            double power = invert * p * (Math.abs(tempHeading - tempTarget) > Math.PI ? (Math.abs(tempHeading > Math.PI ? 2 * Math.PI - tempHeading : tempHeading) + Math.abs(tempTarget > Math.PI ? 2 * Math.PI - tempTarget : tempTarget)) : Math.abs(tempHeading - tempTarget)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setMotorPowers(power, power, -power, -power);
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

//    public Pose2d distanceSensorPose() {
//
//        if(isStopRequested()) return null;
//        double imuHeading = drive.imu.getAngularOrientation().firstAngle;
//        double headingOffsetPlus = imuHeading + Math.toRadians(12.5);
//        double headingOffsetMinus = imuHeading - Math.toRadians(12.5);
//        double anglePlus = Math.toDegrees(Math.abs(headingOffsetPlus)) % 90;
//        anglePlus = anglePlus > 45 ? Math.abs(anglePlus - 90) : anglePlus;
//        double angleMinus = Math.toDegrees(Math.abs(headingOffsetMinus)) % 90;
//        angleMinus = angleMinus > 45 ? Math.abs(angleMinus - 90) : angleMinus;
//        double angleCompensatorPlus = 1 - 0.0002 * anglePlus + 0.0000069 * Math.pow(anglePlus, 2) + 0.00000428 * Math.pow(anglePlus, 3);
//        double angleCompensatorMinus = 1 - 0.0002 * angleMinus + 0.0000069 * Math.pow(angleMinus, 2) + 0.00000428 * Math.pow(angleMinus, 3);
//        double mult = 86, offset = 0.135;
//        double left = mult * (drive.left.getVoltage() - offset);
//        double right = mult * (drive.right.getVoltage() - offset);
//        double back = mult * (drive.back.getVoltage() - offset);
//        double front = mult * (drive.front.getVoltage() - offset);
//
//        telemetry.addLine("Left: " + left);
//        telemetry.addLine("Right: " + right);
//        telemetry.addLine("Back: " + back);
//        telemetry.addLine("Front: " + front);
//
//        //Getting distance from distance sensor to either wall.
//        double leftCos = left * Math.abs(Math.cos(headingOffsetPlus)) * angleCompensatorPlus;
//        double leftSin = left * Math.abs(Math.sin(headingOffsetPlus)) * angleCompensatorPlus;
//        double rightCos = right * Math.abs(Math.cos(headingOffsetMinus)) * angleCompensatorMinus;
//        double rightSin = right * Math.abs(Math.sin(headingOffsetMinus)) * angleCompensatorMinus;
//        double backCos = back * Math.abs(Math.cos(headingOffsetMinus)) * angleCompensatorMinus;
//        double backSin = back * Math.abs(Math.sin(headingOffsetMinus)) * angleCompensatorMinus;
//        double frontCos = front * Math.abs(Math.cos(headingOffsetPlus)) * angleCompensatorPlus;
//        double frontSin = front * Math.abs(Math.sin(headingOffsetPlus)) * angleCompensatorPlus;
//
//        /*telemetry.addLine("Left Cos: " + leftCos);
//        telemetry.addLine("Left Sin: " + leftSin);
//        telemetry.addLine("Right Cos: " + rightCos);
//        telemetry.addLine("Right Sin: " + rightSin);
//        telemetry.addLine("back Cos: " + backCos);
//        telemetry.addLine("back Sin: " + backSin);
//        telemetry.addLine("front Cos: " + frontCos);
//        telemetry.addLine("front Sin: " + frontSin);
//
//         */
//
//        //Assumes radially centered.
//        leftCos += sensorSideOffset * Math.abs(Math.cos(imuHeading));
//        leftSin += sensorSideOffset * Math.abs(Math.sin(imuHeading));
//        rightCos += sensorSideOffset * Math.abs(Math.cos(imuHeading));
//        rightSin += sensorSideOffset * Math.abs(Math.sin(imuHeading));
//        backCos += sensorStrightOffset * Math.abs(Math.cos(imuHeading));
//        backSin += sensorStrightOffset * Math.abs(Math.sin(imuHeading));
//        frontCos += sensorStrightOffset * Math.abs(Math.cos(imuHeading));
//        frontSin += sensorStrightOffset * Math.abs(Math.sin(imuHeading));
//
//        //Get actual X and Y of each position, assuming each input is good, based on heading for every value.
//        leftCos = Math.abs(headingOffsetPlus) < Math.PI / 2 ? -leftCos : leftCos - 142; //Left or right
//        leftSin = headingOffsetPlus < 0 ? -leftSin : leftSin - 142; //Front or back
//        rightCos = Math.abs(headingOffsetMinus) < Math.PI / 2 ? rightCos - 142 : -rightCos; //Right or left
//        rightSin = headingOffsetMinus < 0 ? rightSin - 142 : -rightSin; //Back or front
//        backCos = Math.abs(headingOffsetMinus) < Math.PI / 2 ? backCos - 142 : -backCos; //Front or back
//        backSin = headingOffsetMinus < 0 ? -backSin : backSin - 142; //Left or right
//        frontCos = Math.abs(headingOffsetPlus) < Math.PI / 2 ? -frontCos: frontCos - 142 ; //Back or front
//        frontSin = headingOffsetPlus < 0 ? frontSin  - 142 : -frontSin; //Right or left
//
//        Pose2d pose = drive.getPoseEstimate();
//        double poseX = pose.getX(), poseY = pose.getY();
//        double confidence = 8;
//
//        if(Math.abs(Math.cos(headingOffsetPlus)) > Math.cos(Math.toRadians(20)) || Math.abs(Math.cos(headingOffsetMinus)) > Math.cos(Math.toRadians(20))) {
//            poseY = Math.abs(leftCos - rightCos) < confidence && Math.abs((leftCos + rightCos) / 2 - poseY) < 15 ? (leftCos + rightCos) / 2 : Math.min(Math.abs(leftCos - poseY), Math.abs(rightCos - poseY)) < confidence ? poseY + Math.min(Math.abs(leftCos - poseY), Math.abs(rightCos - poseY)) : poseY;
//            poseX = Math.abs(frontCos - backCos) < confidence && Math.abs((backCos + frontCos) / 2 - poseX) < 15 ? (frontCos + backCos) / 2 : Math.min(Math.abs(backCos - poseX), Math.abs(frontCos - poseX)) < confidence ? poseX + Math.min(Math.abs(backCos - poseX), Math.abs(frontCos - poseX)) : poseX;
//        }
//        else if(Math.abs(Math.sin(headingOffsetPlus)) > Math.cos(Math.toRadians(20)) || Math.abs(Math.sin(headingOffsetMinus)) > Math.cos(Math.toRadians(20))) {
//            poseY = Math.abs(frontSin - backSin) < confidence && Math.abs((backSin + frontSin) / 2 - poseY) < 15 ? (frontSin + backSin) / 2 : Math.min(Math.abs(backSin - poseY), Math.abs(frontSin - poseY)) < confidence ? poseY + Math.min(Math.abs(backSin - poseY), Math.abs(frontSin - poseY)) : poseY;
//            poseX = Math.abs(leftSin - rightSin) < confidence && Math.abs((leftSin + rightSin) / 2 - poseX) < 15 ? (leftSin + rightSin) / 2 : Math.min(Math.abs(leftSin - poseX), Math.abs(rightSin - poseX)) < confidence ? poseX + Math.min(Math.abs(leftSin - poseX), Math.abs(rightSin - poseX)) : poseX;
//        }
//
//        if(poseX < -140) poseX = pose.getX();
//        if(poseY < -100) poseY = pose.getY();
//
//        System.out.println("Old pose: " + pose + ", new pose: " + new Pose2d(poseX, poseY, imuHeading));
//
//        return new Pose2d(poseX, poseY, imuHeading);
//    }

    static class duckScanPipeline extends OpenCvPipeline
    {
        Mat rawMat = new Mat();
        Mat YCRCBMat = new Mat();
        Mat ExtractMat = new Mat();
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
            Point leftUL = new Point(leftX, allY), middleUL = new Point(middleX, allY), rightUL = new Point(rightX, allY);
            rawMat = input;
            //Imgproc.cvtColor(input, YCRCBMat, Imgproc.COLOR_BGR2YCrCb);
            //YCRCBMat = rawMat;
            //Mat rota = Imgproc.getRotationMatrix2D(new Point(160, 120), rotateAngle,1);
            //Imgproc.warpAffine(rawMat, rawMat, rota, new Size(320,240));
            Imgproc.cvtColor(rawMat, YCRCBMat, colorSpace);
            Core.extractChannel(YCRCBMat, ExtractMat, extract);

            double color1 = 0, color2 = 0, color3 = 0;

            for(int i = (int)(leftUL.x); i <= (int)(leftUL.x + sampleWidth); i++){
                for(int j = (int)(leftUL.y);  j <= (int)(leftUL.y + sampleHeight); j++){
                    color1 += ExtractMat.get(j, i)[0];
                }
            }
            color1 /= sampleWidth * sampleHeight;

            for(int i = (int)(middleUL.x); i <= (int)(middleUL.x + sampleWidth); i++){
                for(int j = (int)(middleUL.y);  j <= (int)(middleUL.y + sampleHeight); j++){
                    color2 += ExtractMat.get(j, i)[0];
                }
            }
            color2 /= sampleWidth * sampleHeight;

            for(int i = (int)(rightUL.x); i <= (int)(rightUL.x + sampleWidth); i++){
                for(int j = (int)(rightUL.y);  j <= (int)(rightUL.y + sampleHeight); j++){
                    color3 += ExtractMat.get(j, i)[0];
                }
            }
            color3 /= sampleWidth * sampleHeight;

            boolean leftDuck = color1 > thresh;
            boolean midDuck = color2 > thresh;
            boolean rightDuck = color3 > thresh;

            System.out.println("Color1: " + color1 + ", Color2: " + color2 + "Color3: " + color3);

            if(!leftDuck && !rightDuck) duckLocation = 2;
            else if(leftDuck && !rightDuck) duckLocation = 0;
            else if(!leftDuck && rightDuck) duckLocation = 1;
            else duckLocation = -1;

            Imgproc.rectangle(ExtractMat, leftUL, new Point(leftUL.x + sampleWidth, leftUL.y + sampleHeight), leftDuck ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));
            Imgproc.rectangle(ExtractMat, middleUL, new Point(middleUL.x + sampleWidth, middleUL.y + sampleHeight), midDuck ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));
            Imgproc.rectangle(ExtractMat, rightUL, new Point(rightUL.x + sampleWidth, rightUL.y + sampleHeight), rightDuck ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));

            switch (stageToRenderToViewport){
                case RAW:
                {
                    return rawMat;
                }
                case EXTRACT:
                {
                    return ExtractMat;
                }
                default:
                {
                    return input;
                }
            }
        }

    }
}

