package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.threadedhardware.HardwareThread;
import org.firstinspires.ftc.teamcode.threadedhardware.SampleConfiguration;
import org.firstinspires.ftc.teamcode.threadedhardware.Sequence;

@Config
@TeleOp(name="ScrimTeleOp")
public class TestTeleOp extends LinearOpMode {

    SampleConfiguration config;

    boolean turning = false;

    public static double OPEN = 0.02, CLOSE = 0.69, FLIPDOWN = 1; //0.23 dropper position to for auto lowest level

    public static int[] levels = {0, 660, 1800, 2500};

    private int currentLevel = 0;

    public boolean levelPressed = false;

    public double slidesOffset = 0;

    Thread waitThread, dumpThread;

    HardwareThread hardware;

    @Override
    public void runOpMode() throws InterruptedException {

        config = new SampleConfiguration();
        hardware = new HardwareThread(hardwareMap, config);
        hardware.start();

        double lastHeading = 0, ingesterSpeed = 0;

        config.imu.gettingInput = true;

        sleep(1000);

        config.slides.setPower(-0.5);
        while(!isStopRequested() && config.limit.get()[0] == 0) {}
        config.slides.setPower(0);

        slidesOffset = config.slides.get()[1];

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Limit: ", config.limit.get()[0]);
            telemetry.update();
        }

        config.slides.setTargetPosition(0);
        config.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        while(!isStopRequested()) {

            hardware.waitForCycle();

            config.ingest.setPower(ingesterSpeed);
            config.spinner.setPower(gamepad2.left_stick_y);
            config.preIngest.setPower(ingesterSpeed * 0.6);

            if(gamepad1.back) config.flipdown.set(FLIPDOWN);

            if(gamepad1.a || gamepad2.a) ingesterSpeed = 1;
            else if(gamepad1.y || gamepad2.y) ingesterSpeed = -1;
            else if(gamepad1.back || gamepad2.back) ingesterSpeed = 0;

            double imuHeading = config.imu.get()[0];
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

            double tempPos = config.slides.get()[1] - slidesOffset;

            double pow = tempPos > levels[currentLevel] ? -1 : 1;

            if(Math.abs(tempPos - levels[currentLevel]) < 50 || (pow == -1 && config.limit.get()[0] == 1)) pow = 0.05;

            //config.slides.setPower((!gamepad2.left_bumper ? pow : (config.limit.get()[0] != 1 || gamepad2.right_stick_y > 0) ? gamepad2.right_stick_y : 0));
            double pow1 = Math.abs(gamepad2.right_stick_y) < 0.3 ? 0 : gamepad2.right_stick_y;
            if((tempPos <= 0 && pow1 < 0) || config.limit.get()[0] == 1) pow1 = 0;

            //config.slides.setPower(pow1 == 0 ? pow : pow1);
            config.slides.setTargetPosition(levels[currentLevel]);
            config.slides.setPower(1);

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

            setPower(speed * x, -speed * y, speed * a + power);

            telemetry.addData("Heading: ", imuHeading);
            telemetry.addData("Power: ", power);
            telemetry.addData("Last Heading: ", lastHeading);
            telemetry.addData("Level: ", currentLevel);
            telemetry.addData("Slide Height: ", tempPos);
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