package org.firstinspires.ftc.teamcode.threadedhardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ThreadedCRServo implements Active {

    private CRServo servo;
    private int partNum;

    private ActionThread thread = new NullThread();

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateHardware = true;

    //Value that the servo is set to
    protected volatile double runVal = 0;

    //Constructors

    public ThreadedCRServo(HardwareMap hwMap, String objectName){
        servo = hwMap.get(CRServo.class, objectName);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    //Interface methods

    public synchronized void set(double position) {
        runVal = position;
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    public void setHardware() {
        servo.setPower(runVal);
    }

    public void updateMode() {} //As of now, no reason for this in servo.

    @Override
    public double getRunVal() {
        return runVal;
    }

    public void getHardware() {
        hardwareVals = new double[]{servo.getPower()};
        updateHardware = !updateHardware;
    }

    public void endThreads() {
        thread.Stop();
    }
}
