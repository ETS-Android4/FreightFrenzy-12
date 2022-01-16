package org.firstinspires.ftc.teamcode.threadedhardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ThreadedVoltageSensor implements Sensor, VoltageSensor {

    private VoltageSensor sensor;
    private int partNum;

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateVals = true;

    //Maybe add a thread to stop when a condition is met
    private ActionThread thread = new NullThread(); //Not in use at the moment, but likely to be used in future updates.

    //Constructors

    //Not sure if this works
    public ThreadedVoltageSensor(HardwareMap hwMap, String objectName) {
        sensor = hwMap.voltageSensor.get(objectName);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    public ThreadedVoltageSensor(HardwareMap hwMap) {
        sensor = hwMap.voltageSensor.iterator().next();
        this.partNum = hardware.size();
        hardware.add(this);
    }

    //Interface methods

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    //Binarizes boolean for easier math
    public void getHardware() {
        hardwareVals = new double[]{sensor.getVoltage()};

        //Setting a volatile variable saves all values to main memory
        updateVals = !updateVals;
    }

    public void endThreads() {
        //thread.Stop();
    }

    public double getVoltage() {
        return hardwareVals[0];
    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
