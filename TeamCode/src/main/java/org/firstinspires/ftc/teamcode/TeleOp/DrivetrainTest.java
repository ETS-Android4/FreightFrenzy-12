package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.CVConfig;

@Config
@TeleOp
public class DrivetrainTest extends LinearOpMode {

    CVConfig config;
    public static double armPos = 0, lidPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        config = new CVConfig();
        config.configure(hardwareMap);

        waitForStart();

        while(!isStopRequested()) {
            setPower(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            config.arm.setPosition(armPos);
            config.lid.setPosition(lidPos);
        }
    }

    public void setPower(double x, double y, double a){
        config.backLeft.setPower(-x + y + a);
        config.frontLeft.setPower(x + y + a);
        config.frontRight.setPower(-x + y - a);
        config.backRight.setPower(x + y - a);
    }
}
