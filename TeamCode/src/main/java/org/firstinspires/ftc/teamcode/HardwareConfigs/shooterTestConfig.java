package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class shooterTestConfig {

    HardwareMap hwMap;

    public Servo servo;
    public DcMotorImplEx motor;
    public DigitalChannelImpl limit;

    public HardwareMap Configure(HardwareMap ahwMap) {

        hwMap = ahwMap;

        motor = hwMap.get(DcMotorImplEx.class,"spinner");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //servo = hwMap.get(Servo.class, "dropper");
        limit = hwMap.get(DigitalChannelImpl.class, "limit");
        //loader = hwMap.get(Servo.class, "servo");

        return hwMap;
    }
}
