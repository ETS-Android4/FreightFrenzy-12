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

    public static double p = 0.00001;

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
            while(Math.abs(power) > 0.05) {
                HardwareThread.waitForCycle();
                power = p * (HubVisionPipeline.centerPointHub.x - 320);
                setPower(0, 0, power);
            }
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

            if(gamepad1.dpad_left && !testThread.isAlive()) testThread.start();

            if(!testThread.isAlive() && !hubThread.isAlive()) setPower(speed * x, -speed * y, speed * a + power);

            telemetry.addData("Heading: ", imuHeading);
            telemetry.addData("Power: ", config.backLeft.get()[0]);
            //telemetry.addData("Last Heading: ", lastHeading);
            //telemetry.addData("Level: ", currentLevel);
            //telemetry.addData("Slide Height: ", tempPos);
            //telemetry.addData("Drivetrain Current Draw: ");
            //telemetry.addData("Limit: ", config.limit.get()[0]);
           // telemetry.addData("Expected Height: ", levels[currentLevel]);
            telemetry.update();
        }

        hardware.Stop();
    }

    public void setPower(double x, double y, double a){
        config.backLeft.setPower(-x + y + a);
        config.frontLeft.setPower(x + y + a);
        config.frontRight.setPower(-x + y - a);
        config.backRight.setPower(x + y - a);
    }
}