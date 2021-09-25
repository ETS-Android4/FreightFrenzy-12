package org.firstinspires.ftc.teamcode.threadedhardware;

public interface Active extends ThreadedHardware {

    void set(double value);
    void setHardware();

    double getRunVal();
}
