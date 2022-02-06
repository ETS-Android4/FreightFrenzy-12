package org.firstinspires.ftc.teamcode.threadedhardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class SampleConfiguration implements Configuration {

    //Expansion hubs, which control access to physical hardware.
    private List<LynxModule> allHubs;

    public ThreadedMotor backLeft, frontLeft, frontRight, backRight, spinner, slides, ingest, preIngest;

    public ThreadedServo dropper, flipdown;

    public ThreadedDigitalSensor limit;

    public ThreadedIMU imu; //A gyroscope in the expansion hubs (inertial measurement unit)

    public void Configure(HardwareMap hwMap){
        //Create all hardware objects here.

        //For analog sensors, you will need to create and pass a lambda notation that interprets the raw voltage received from the sensor.
        //This is an example, and is what we used for our Maxbotix distance sensors.
        ThreadedAnalogSensor.InterpretVoltage distance = ((double voltage, double max) -> 87.4 * (voltage - 0.138));

        hardware.clear(); //Hardware is an ArrayList on the Hardware interface, which the Configuration interface extends.
        backLeft = new ThreadedMotor(hwMap, "back_left_motor");
        frontLeft = new ThreadedMotor(hwMap, "front_left_motor");
        frontRight = new ThreadedMotor(hwMap, "front_right_motor");
        backRight = new ThreadedMotor(hwMap, "back_right_motor");
        ingest = new ThreadedMotor(hwMap, "ingest");
        preIngest = new ThreadedMotor(hwMap, "preingest");
        spinner = new ThreadedMotor(hwMap, "spinner");
        slides = new ThreadedMotor(hwMap, "slides");
        dropper = new ThreadedServo(hwMap, "dropper");
        flipdown = new ThreadedServo(hwMap, "flipdown");
        limit = new ThreadedDigitalSensor(hwMap, "limit");
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ingest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        preIngest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = new ThreadedIMU(hwMap);

        frontLeft.reverse(true);
        backLeft.reverse(true);
        ingest.reverse(true);

        allHubs = hwMap.getAll(LynxModule.class);

        setBulkCachingManual(true);
    }

    public void setBulkCachingManual(boolean manual){
        //Manual bulk caching lets me control exactly when to read from the hardware, which lets me optimize my loop times.
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(manual ? LynxModule.BulkCachingMode.MANUAL : LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            if(module.getBulkCachingMode() == LynxModule.BulkCachingMode.MANUAL) {
                module.clearBulkCache();
                module.getBulkData();
            }
        }
    }
}
