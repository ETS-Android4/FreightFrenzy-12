package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.gson.internal.$Gson$Preconditions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Vision.HubVisionPipeline;
import org.firstinspires.ftc.teamcode.threadedhardware.Hardware;
import org.firstinspires.ftc.teamcode.threadedhardware.HardwareThread;
import org.firstinspires.ftc.teamcode.threadedhardware.RoadRunnerConfiguration;
import org.firstinspires.ftc.teamcode.threadedhardware.SampleConfiguration;
import org.firstinspires.ftc.teamcode.threadedhardware.Sequence;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.Vision.HubVisionPipeline.hubScanPipeline;

@Config
@TeleOp(name="RRTeleOp")
public class RRTeleOp extends LinearOpMode {

    RoadRunnerConfiguration config;

    public final int rows = 640;
    public final int cols = 480;
    public static int g;
    public static int exp;

    boolean turning = false;

    public static double sensorSideOffset = 0, sensorStrightOffset = 0;

    public static double p = 0.00009, targetX = 150, yPow = 1, maximumHubWidth = 130, startingSlowDownWidth = 80, slowDownFactorDenominator = 10;

    public static double OPEN = 0.02, CLOSE = 0.69, FLIPDOWN = 1, x = 20, y = 60; //0.23 dropper position to for auto lowest level

    public static int[] levels = {0, 660, 1800, 2500};

    private int currentLevel = 0;

    public boolean levelPressed = false;

    public double slidesOffset = 0, imuHeading = 0;

    OpenCvWebcam webCam;

    Thread waitThread, dumpThread, testThread, hubThread;

    HardwareThread hardware;

    @Override
    public void runOpMode() throws InterruptedException {

        config = new RoadRunnerConfiguration(hardwareMap);
        hardware = new HardwareThread(hardwareMap, config);
        hardware.start();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        //1 camera at the moment.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        webCam.openCameraDevice();//open camera
        webCam.setPipeline(new HubVisionPipeline.hubScanPipeline());
        webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        FtcDashboard.getInstance().startCameraStream(webCam, 0);

        ExposureControl exposure = webCam.getExposureControl();
        GainControl gain = webCam.getGainControl();
        exposure.setMode(ExposureControl.Mode.Manual);
        g = gain.getGain();
        exp = (int) exposure.getExposure(TimeUnit.MILLISECONDS);
        exp = 14;
        g = 0;

        gain.setGain(g);
        exposure.setExposure(exp, TimeUnit.MILLISECONDS);

        double lastHeading = 0, ingesterSpeed = 0;

        config.imu.gettingInput = true;

        sleep(1000);

        //config.slides.setPower(-0.5);
        //while(!isStopRequested() && config.limit.get()[0] == 0) {}
        //config.slides.setPower(0);

        //slidesOffset = config.slides.get()[1];

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Left: ", config.left.get()[1]);
            telemetry.addData("Right: ", config.right.get()[0]);
            telemetry.addData("Front: ", config.front.get()[0]);
            telemetry.update();
        }

        waitForStart();

        Sequence waitMillis = new Sequence(() -> {
            sleep(500);
            turning = false;
        }, null);
        waitThread = new Thread(waitMillis);

        Sequence dump = new Sequence(() -> {
            config.dropper.set(OPEN);
            sleep(650);
            config.dropper.set(CLOSE);
            currentLevel = 0;
        });
        dumpThread = new Thread(dump);

        Sequence forward = new Sequence(() -> {
            config.setPoseEstimate(new Pose2d(0, 0));
            config.turn(-imuHeading);
            double wallX = config.left.get()[0];
            Trajectory traj = config.trajectoryBuilder(config.getPoseEstimate())
                    .strafeLeft(wallX - 1)
                    .build();
            config.followTrajectory(traj);

            Trajectory traj2 = config.trajectoryBuilder(config.getPoseEstimate())
                    .forward(y)
                    .build();
            config.followTrajectory(traj2);
        });
        testThread = new Thread(forward);

        Sequence turnToHub = new Sequence(() -> {
            double power = 1;
            double x = HubVisionPipeline.getCenterPointHub().x;
            double yPower = -0.4;
            double initialW = HubVisionPipeline.width;
            while(HubVisionPipeline.width < (maximumHubWidth-15) && x != -1 && yPower < 0) {
                HardwareThread.waitForCycle();
                double inputW = HubVisionPipeline.width;
                power = p * (targetX - x);
                double slowDownFactor = Math.max(Math.pow((maximumHubWidth - inputW), 2) / Math.pow((maximumHubWidth - initialW), 2), 0);
                yPower = -slowDownFactor*yPow;
                //yPower = Math.min(Math.abs(power) - slowDownFactor*yPow, 0);
                setPower(0, yPower, power);
                x = HubVisionPipeline.getCenterPointHub().x;
                System.out.println("This piece of shit has a power of " + power + " and an x of " + x);
            }
            setPower(0, 0, 0);
        });
        hubThread = new Thread(turnToHub);

        while(!isStopRequested()) {

            hardware.waitForCycle();

            //config.ingest.setPower(ingesterSpeed);
            //config.spinner.setPower(gamepad2.left_stick_y);
            //config.preIngest.setPower(ingesterSpeed * 0.6);

            if(gamepad1.back) config.flipdown.set(FLIPDOWN);

            if(gamepad1.a || gamepad2.a) ingesterSpeed = 1;
            else if(gamepad1.y || gamepad2.y) ingesterSpeed = -1;
            else if(gamepad1.back || gamepad2.back) ingesterSpeed = 0;

            if(gamepad1.left_stick_button && !hubThread.isAlive()) hubThread.start();

            imuHeading = config.imu.get()[0];
            double tempHeading = imuHeading;
            double tempTarget = lastHeading;
            if(tempHeading < 0) tempHeading += 2 * Math.PI;
            if(lastHeading < 0) tempTarget += 2 * Math.PI;
            double invert = lastHeading - imuHeading;
            if(invert > Math.PI) invert -= 2 * Math.PI;
            else if(invert < -Math.PI) invert += 2 * Math.PI;
            invert = invert < 0 ? 1 : -1;
            double power = invert * 0.8 * (Math.abs(tempHeading - tempTarget) > Math.PI ? (Math.abs(tempHeading > Math.PI ? 2 * Math.PI - tempHeading : tempHeading) + Math.abs(tempTarget > Math.PI ? 2 * Math.PI - tempTarget : tempTarget)) : Math.abs(tempHeading - tempTarget)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            if(Math.abs(power) < 0.05) power *= 0.5;
            power = 0;
            if(Math.abs(gamepad1.right_stick_x) > 0.1) turning = true;

            else if(turning && !waitThread.isAlive()) waitThread.start();

            //config.slides.setPower(gamepad2.left_stick_y);
            if((gamepad1.dpad_up || gamepad2.dpad_up) && !levelPressed) {
                currentLevel += currentLevel < 3 ? 1 : 0;
                levelPressed = true;
            }
            else if((gamepad1.dpad_down || gamepad2.dpad_down) && !levelPressed) {
                currentLevel -= currentLevel > 0 ? 1 : 0;
                levelPressed = true;
            }
            else if(!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_up) levelPressed = false;

            //double tempPos = config.slides.get()[1] - slidesOffset;

            //double pow = tempPos > levels[currentLevel] ? -1 : 1;

            //if(Math.abs(tempPos - levels[currentLevel]) < 150 || (pow == -1 && config.limit.get()[0] == 1)) pow = 0;
            //if(Math.abs(tempPos - levels[currentLevel]) < 150) pow = 0.001;

            //config.slides.setPower((!gamepad2.left_bumper ? pow : (config.limit.get()[0] != 1 || gamepad2.right_stick_y > 0) ? gamepad2.right_stick_y : 0));
            //double pow1 = Math.abs(gamepad2.right_stick_y) < 0.3 ? 0 : gamepad2.right_stick_y;
            //if((tempPos <= 0 && pow1 < 0) || config.limit.get()[0] == 1) pow1 = 0;

            //config.slides.setPower(pow);

            if(turning || gamepad1.left_trigger > 0.3 || gamepad1.right_trigger > 0.3) {
                lastHeading = imuHeading;
                power = 0;
            }

            if(dumpThread.isAlive());
            else if(gamepad1.start) dumpThread.start();
            else if(gamepad1.x) config.dropper.set(CLOSE);
            else if(gamepad1.b) config.dropper.set(OPEN);

            double speed = gamepad1.right_bumper ? 0.3 : 1;
            double x = 0.6 * -gamepad1.left_trigger + 0.6 * gamepad1.right_trigger + gamepad1.left_stick_x, y = gamepad1.left_stick_y, a = gamepad1.right_stick_x;

            //if(gamepad1.dpad_left && !testThread.isAlive()) testThread.start();

            if(!testThread.isAlive() && !hubThread.isAlive()) setPower(speed * x, -speed * y, speed * a + power);



            telemetry.addData("Heading: ", imuHeading);
            telemetry.addData("Power: ", config.backLeft.get()[0]);
            telemetry.addData("Width: ", HubVisionPipeline.width);
            telemetry.addData("HubCenterPoint: ", HubVisionPipeline.getCenterPointHub().x);
            //telemetry.addData("Last Heading: ", lastHeading);
            //telemetry.addData("Level: ", currentLevel);
            //telemetry.addData("Slide Height: ", tempPos);
            //telemetry.addData("Drivetrain Current Draw: ");
            //telemetry.addData("Limit: ", config.limit.get()[0]);
            //telemetry.addData("Expected Height: ", levels[currentLevel]);
            telemetry.update();
        }

        hardware.Stop();
    }

    public Pose2d sensorPoseAnalog() {
        //Do not call this in a situation where distance sensors could hit the same wall
        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).

        //CURRENTLY THE SYSTEM PERPENDICULAR TO WALLS since I would like to test whether the system works before I do fancy stuff.

        //As of April 6, the sensors appear to have a tolerance for 40 degrees either side, with distance not seeming to matter. I would go 30 degrees to be safe.

        //Sensor max is 25 degrees, set to 12.5 degrees on the robot.

        //drive.imu.gettingInput = true;
        if(isStopRequested()) return null;
        double imuHeading = config.imu.get()[0];
        double left = -1; //config.left.get()[0];
        double right = config.right.get()[0];
        double front = config.front.get()[0];

        if(left < 0 || left > 60) left = -1;
        if(right < 0 || right > 60) right = -1;
        if(front < 0 || front > 60) front = -1;

        double cos = Math.abs(Math.cos(imuHeading));
        double sin = Math.abs(Math.sin(imuHeading));

        //Getting distance from distance sensor to either wall.
        double leftCos = left * cos;
        double leftSin = left * sin;
        double rightCos = right * cos;
        double rightSin = right * sin;
        double frontCos = front * cos;
        double frontSin = front * sin;

        //Assumes radially centered.
        leftCos += sensorSideOffset * Math.abs(Math.cos(imuHeading));
        leftSin += sensorSideOffset * Math.abs(Math.sin(imuHeading));
        rightCos += sensorSideOffset * Math.abs(Math.cos(imuHeading));
        rightSin += sensorSideOffset * Math.abs(Math.sin(imuHeading));
        frontCos += sensorStrightOffset * Math.abs(Math.cos(imuHeading));
        frontSin += sensorStrightOffset * Math.abs(Math.sin(imuHeading));

        //Get actual X and Y of each position, assuming each input is good, based on heading for every value.
        leftCos = Math.abs(imuHeading) < Math.PI / 2 ? -leftCos : leftCos - 94; //Left or right
        leftSin = imuHeading < 0 ? -leftSin : leftSin - 142; //Front or back
        rightCos = Math.abs(imuHeading) < Math.PI / 2 ? rightCos - 94 : -rightCos; //Right or left
        rightSin = imuHeading < 0 ? rightSin - 142 : -rightSin; //Back or front
        frontCos = Math.abs(imuHeading) < Math.PI / 2 ? frontCos - 142 : -frontCos; //Front or back
        frontSin = imuHeading < 0 ? -frontSin : frontSin - 94; //Left or right

        Pose2d pose = config.getPoseEstimate();
        double poseX = pose.getX(), poseY = pose.getY();
        double confidence = 6;

        if(Math.abs(Math.cos(imuHeading)) > Math.cos(Math.toRadians(20)) || Math.abs(Math.cos(imuHeading)) > Math.cos(Math.toRadians(20))) {
            poseY = Math.abs(leftCos - rightCos) < confidence && Math.abs((leftCos + rightCos) / 2 - poseY) < 15 ? (leftCos + rightCos) / 2 : Math.min(Math.abs(leftCos - poseY), Math.abs(rightCos - poseY)) < confidence ? poseY + Math.min(Math.abs(leftCos - poseY), Math.abs(rightCos - poseY)) : poseY;
            poseX = Math.abs(frontSin - frontCos) < confidence && Math.abs((frontCos + frontSin) / 2 - poseX) < 15 ? (frontSin + frontCos) / 2 : Math.min(Math.abs(frontCos - poseX), Math.abs(frontSin - poseX)) < confidence ? poseX + Math.min(Math.abs(frontCos - poseX), Math.abs(frontSin - poseX)) : poseX;
        }
        else if(Math.abs(Math.sin(imuHeading)) > Math.cos(Math.toRadians(20)) || Math.abs(Math.sin(imuHeading)) > Math.cos(Math.toRadians(20))) {
            poseY = Math.abs(frontCos - frontSin) < confidence && Math.abs((frontSin + frontCos) / 2 - poseY) < 15 ? (frontCos + frontSin) / 2 : Math.min(Math.abs(frontSin - poseY), Math.abs(frontCos - poseY)) < confidence ? poseY + Math.min(Math.abs(frontSin - poseY), Math.abs(frontCos - poseY)) : poseY;
            poseX = Math.abs(leftSin - rightSin) < confidence && Math.abs((leftSin + rightSin) / 2 - poseX) < 15 ? (leftSin + rightSin) / 2 : Math.min(Math.abs(leftSin - poseX), Math.abs(rightSin - poseX)) < confidence ? poseX + Math.min(Math.abs(leftSin - poseX), Math.abs(rightSin - poseX)) : poseX;
        }

        if(poseX < -140) poseX = pose.getX();
        if(poseY < -100) poseY = pose.getY();

        System.out.println("Old pose: " + pose + ", new pose: " + new Pose2d(poseX, poseY, imuHeading));

        return new Pose2d(poseX, poseY, imuHeading);
    }

    public void setPower(double x, double y, double a){
        config.backLeft.setPower(-x + y + a);
        config.frontLeft.setPower(x + y + a);
        config.frontRight.setPower(-x + y - a);
        config.backRight.setPower(x + y - a);
    }
}