package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfigs.shooterTestConfig;

@Config
@TeleOp
public class IngestTest extends LinearOpMode {

    public static boolean motorOn = true, servoOn = false;
    public static double pos = 1;
    public static double power = 0.8, motorPos = 500, durationMilli = 1750, rampP = -0.0008, rampF = -0.3;

    //Duck carousel notes: good ducks can go about 0.1 faster on p1 and p2, and have a shorter s2 delay. However, bad ducks be scuffed. Maybe put bad ducks
    //on their side from the start to roll in? They turn over quickly regardless.

    @Override
    public void runOpMode() throws InterruptedException {
        shooterTestConfig config = new shooterTestConfig();
        config.Configure(hardwareMap);
        //config.motor.setTargetPosition(1000);
        //config.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        ElapsedTime time = new ElapsedTime();

        while(!isStopRequested()) {
            //System.out.println("Limit: " + config.limit.getState());
            telemetry.addLine("Limit: " + config.limit.getState());
            telemetry.update();
            if(motorOn) {
                if(gamepad2.left_stick_button && time.milliseconds() > durationMilli){
                    time.reset();
                }
                if(time.milliseconds() < durationMilli) config.motor.setPower(rampF + rampP * time.milliseconds());
                else config.motor.setPower(0);
                telemetry.addLine("Encoder position: " + config.motor.getCurrentPosition());
            }
            //if(servoOn) config.servo.setPosition(pos);
        }


    }
}
