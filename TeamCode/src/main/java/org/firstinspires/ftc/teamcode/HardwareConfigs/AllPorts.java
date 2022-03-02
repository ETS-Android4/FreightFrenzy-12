package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

@Config
public class AllPorts {

    HardwareMap hwMap;

    public static boolean S00 = false, S01 = false, S02 = false, S03 = false, S04 = false, S05 = false, S10 = false, S11 = false, S12 = false, S13 = false, S14 = false, S15 = false;
    public static boolean C00 = false, C01 = false, C02 = false, C03 = false, C04 = false, C05 = false, C10 = false, C11 = false, C12 = false, C13 = false, C14 = false, C15 = false;
    public static boolean M00 = false, M01 = false, M02 = false, M03 = false, M10 = false, M11 = false, M12 = false, M13 = false;
    public static boolean D00 = false, D01 = false, D02 = false, D03 = false, D04 = false, D05 = false, D06 = false, D07 = false, D10 = false, D11 = false, D12 = false, D13 = false, D14 = false, D15 = false, D16 = false, D17 = false;
    public static boolean A00 = false, A01 = false, A02 = false, A03 = false, A10 = false, A11 = false, A12 = false, A13 = false;
    public static boolean IMU = false;

    public static Servo[] servos = new Servo[12];
    public static CRServo[] crservos = new CRServo[12];
    public static DcMotorImplEx[] motors = new DcMotorImplEx[8];
    public static DigitalChannelImpl[] digitals = new DigitalChannelImpl[16];
    public static AnalogInput[] analogs = new AnalogInput[8];
    public static BNO055IMU imu;

    public HardwareMap Configure(HardwareMap ahwMap) {

        hwMap = ahwMap;

        if(S00) servos[0] = hwMap.get(Servo.class, "s00");
        if(S01) servos[1] = hwMap.get(Servo.class, "s01");
        if(S02) servos[2] = hwMap.get(Servo.class, "s02");
        if(S03) servos[3] = hwMap.get(Servo.class, "s03");
        if(S04) servos[4] = hwMap.get(Servo.class, "s04");
        if(S05) servos[5] = hwMap.get(Servo.class, "s05");
        if(S10) servos[6] = hwMap.get(Servo.class, "s10");
        if(S11) servos[7] = hwMap.get(Servo.class, "s11");
        if(S12) servos[8] = hwMap.get(Servo.class, "s12");
        if(S13) servos[9] = hwMap.get(Servo.class, "s13");
        if(S14) servos[10] = hwMap.get(Servo.class, "s14");
        if(S15) servos[11] = hwMap.get(Servo.class, "s15");
        if(C00) crservos[0] = hwMap.get(CRServo.class, "c00");
        if(C01) crservos[1] = hwMap.get(CRServo.class, "c01");
        if(C02) crservos[2] = hwMap.get(CRServo.class, "c02");
        if(C03) crservos[3] = hwMap.get(CRServo.class, "c03");
        if(C04) crservos[4] = hwMap.get(CRServo.class, "c04");
        if(C05) crservos[5] = hwMap.get(CRServo.class, "c05");
        if(C10) crservos[6] = hwMap.get(CRServo.class, "c10");
        if(C11) crservos[7] = hwMap.get(CRServo.class, "c11");
        if(C12) crservos[8] = hwMap.get(CRServo.class, "c12");
        if(C13) crservos[9] = hwMap.get(CRServo.class, "c13");
        if(C14) crservos[10] = hwMap.get(CRServo.class, "c14");
        if(C15) crservos[11] = hwMap.get(CRServo.class, "c15");
        motors[0] = hwMap.get(DcMotorImplEx.class, "m00");
        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1] = hwMap.get(DcMotorImplEx.class, "m01");
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[2] = hwMap.get(DcMotorImplEx.class, "m02");
        motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[3] = hwMap.get(DcMotorImplEx.class, "m03");
        motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[4] = hwMap.get(DcMotorImplEx.class, "m10");
        motors[4].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[4].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[5] = hwMap.get(DcMotorImplEx.class, "m11");
        motors[5].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[5].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[6] = hwMap.get(DcMotorImplEx.class, "m12");
        motors[6].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[6].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[7] = hwMap.get(DcMotorImplEx.class, "m13");
        motors[7].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[7].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        digitals[0] = hwMap.get(DigitalChannelImpl.class, "d00");
        digitals[1] = hwMap.get(DigitalChannelImpl.class, "d01");
        digitals[2] = hwMap.get(DigitalChannelImpl.class, "d02");
        digitals[3] = hwMap.get(DigitalChannelImpl.class, "d03");
        digitals[4] = hwMap.get(DigitalChannelImpl.class, "d04");
        digitals[5] = hwMap.get(DigitalChannelImpl.class, "d05");
        digitals[6] = hwMap.get(DigitalChannelImpl.class, "d06");
        digitals[7] = hwMap.get(DigitalChannelImpl.class, "d07");
        digitals[8] = hwMap.get(DigitalChannelImpl.class, "d10");
        digitals[9] = hwMap.get(DigitalChannelImpl.class, "d11");
        digitals[10] = hwMap.get(DigitalChannelImpl.class, "d12");
        digitals[11] = hwMap.get(DigitalChannelImpl.class, "d13");
        digitals[12] = hwMap.get(DigitalChannelImpl.class, "d14");
        digitals[13] = hwMap.get(DigitalChannelImpl.class, "d15");
        digitals[14] = hwMap.get(DigitalChannelImpl.class, "d16");
        digitals[15] = hwMap.get(DigitalChannelImpl.class, "d17");
        analogs[0] = hwMap.get(AnalogInput.class, "a00");
        analogs[1] = hwMap.get(AnalogInput.class, "a01");
        analogs[2] = hwMap.get(AnalogInput.class, "a02");
        analogs[3] = hwMap.get(AnalogInput.class, "a03");
        analogs[4] = hwMap.get(AnalogInput.class, "a10");
        analogs[5] = hwMap.get(AnalogInput.class, "a11");
        analogs[6] = hwMap.get(AnalogInput.class, "a12");
        analogs[7] = hwMap.get(AnalogInput.class, "a13");
        if(IMU) imu = hwMap.get(BNO055IMU.class, "imu");

        return hwMap;
    }
}
