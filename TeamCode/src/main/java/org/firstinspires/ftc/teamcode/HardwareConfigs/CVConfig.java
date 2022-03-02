package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class CVConfig {

    public DcMotorImplEx backLeft;
    public DcMotorImplEx backRight;
    public DcMotorImplEx frontLeft;
    public DcMotorImplEx frontRight;
    public Servo arm;
    public Servo lid;

    public static boolean blReverse = true, flReverse = true, brReverse = false, frReverse = false;

    public void configure(HardwareMap hwMap) {
        backLeft = hwMap.get(DcMotorImplEx.class, "back_left_motor");
        backRight = hwMap.get(DcMotorImplEx.class, "back_right_motor");
        frontLeft = hwMap.get(DcMotorImplEx.class, "front_left_motor");
        frontRight = hwMap.get(DcMotorImplEx.class, "front_right_motor");
        arm = hwMap.get(Servo.class, "arm");
        lid = hwMap.get(Servo.class, "lid");

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(blReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(flReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(brReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(frReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
