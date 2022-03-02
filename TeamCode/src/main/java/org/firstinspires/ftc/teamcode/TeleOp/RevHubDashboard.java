package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.HardwareConfigs.AllPorts.analogs;
import static org.firstinspires.ftc.teamcode.HardwareConfigs.AllPorts.crservos;
import static org.firstinspires.ftc.teamcode.HardwareConfigs.AllPorts.digitals;
import static org.firstinspires.ftc.teamcode.HardwareConfigs.AllPorts.servos;
import static org.firstinspires.ftc.teamcode.HardwareConfigs.AllPorts.motors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.AllPorts;

@Config
@TeleOp
public class RevHubDashboard extends LinearOpMode {

    public static double power = 0;
    public static double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        AllPorts config = new AllPorts();
        config.Configure(hardwareMap);

        waitForStart();

        while(!isStopRequested()) {
            if(AllPorts.S00) servos[0].setPosition(position);
            if(AllPorts.S01) servos[1].setPosition(position);
            if(AllPorts.S02) servos[2].setPosition(position);
            if(AllPorts.S03) servos[3].setPosition(position);
            if(AllPorts.S04) servos[4].setPosition(position);
            if(AllPorts.S05) servos[5].setPosition(position);
            if(AllPorts.S10) servos[6].setPosition(position);
            if(AllPorts.S11) servos[7].setPosition(position);
            if(AllPorts.S12) servos[8].setPosition(position);
            if(AllPorts.S13) servos[9].setPosition(position);
            if(AllPorts.S14) servos[10].setPosition(position);
            if(AllPorts.S15) servos[11].setPosition(position);
            if(AllPorts.C00) crservos[0].setPower(power);
            if(AllPorts.C01) crservos[1].setPower(power);
            if(AllPorts.C02) crservos[2].setPower(power);
            if(AllPorts.C03) crservos[3].setPower(power);
            if(AllPorts.C04) crservos[4].setPower(power);
            if(AllPorts.C05) crservos[5].setPower(power);
            if(AllPorts.C10) crservos[6].setPower(power);
            if(AllPorts.C11) crservos[7].setPower(power);
            if(AllPorts.C12) crservos[8].setPower(power);
            if(AllPorts.C13) crservos[9].setPower(power);
            if(AllPorts.C14) crservos[10].setPower(power);
            if(AllPorts.C15) crservos[11].setPower(power);
            if(AllPorts.M00) setMotor(0);
            if(AllPorts.M01) setMotor(1);
            if(AllPorts.M02) setMotor(2);
            if(AllPorts.M03) setMotor(3);
            if(AllPorts.M10) setMotor(4);
            if(AllPorts.M11) setMotor(5);
            if(AllPorts.M12) setMotor(6);
            if(AllPorts.M13) setMotor(7);
            if(AllPorts.D00) telemetry.addLine("Digital 0: " + digitals[0].getState());
            if(AllPorts.D01) telemetry.addLine("Digital 1: " + digitals[1].getState());
            if(AllPorts.D02) telemetry.addLine("Digital 2: " + digitals[2].getState());
            if(AllPorts.D03) telemetry.addLine("Digital 3: " + digitals[3].getState());
            if(AllPorts.D04) telemetry.addLine("Digital 4: " + digitals[4].getState());
            if(AllPorts.D05) telemetry.addLine("Digital 5: " + digitals[5].getState());
            if(AllPorts.D06) telemetry.addLine("Digital 6: " + digitals[6].getState());
            if(AllPorts.D07) telemetry.addLine("Digital 7: " + digitals[7].getState());
            if(AllPorts.D10) telemetry.addLine("Digital 8: " + digitals[8].getState());
            if(AllPorts.D11) telemetry.addLine("Digital 9: " + digitals[9].getState());
            if(AllPorts.D12) telemetry.addLine("Digital 10: " + digitals[10].getState());
            if(AllPorts.D13) telemetry.addLine("Digital 11: " + digitals[11].getState());
            if(AllPorts.D14) telemetry.addLine("Digital 12: " + digitals[12].getState());
            if(AllPorts.D15) telemetry.addLine("Digital 13: " + digitals[13].getState());
            if(AllPorts.D16) telemetry.addLine("Digital 14: " + digitals[14].getState());
            if(AllPorts.D17) telemetry.addLine("Digital 15: " + digitals[15].getState());
            if(AllPorts.A00) telemetry.addLine("Analog 0: " + analogs[0].getVoltage());
            if(AllPorts.A01) telemetry.addLine("Analog 1: " + analogs[1].getVoltage());
            if(AllPorts.A02) telemetry.addLine("Analog 2: " + analogs[2].getVoltage());
            if(AllPorts.A03) telemetry.addLine("Analog 3: " + analogs[3].getVoltage());
            if(AllPorts.A10) telemetry.addLine("Analog 4: " + analogs[4].getVoltage());
            if(AllPorts.A11) telemetry.addLine("Analog 5: " + analogs[5].getVoltage());
            if(AllPorts.A12) telemetry.addLine("Analog 6: " + analogs[6].getVoltage());
            if(AllPorts.A13) telemetry.addLine("Analog 7: " + analogs[7].getVoltage());
            telemetry.update();
        }
    }

    private void setMotor(int index) {
        motors[index].setPower(power);
        telemetry.addLine("Motor " + index + " Position: " + motors[index].getCurrentPosition());
    }
}
