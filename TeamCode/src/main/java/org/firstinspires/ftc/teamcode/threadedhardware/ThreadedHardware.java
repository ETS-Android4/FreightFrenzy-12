package org.firstinspires.ftc.teamcode.threadedhardware;

public interface ThreadedHardware extends Hardware {

    int getPartNum();
    double[] get();
    void getHardware();
    void endThreads();
}
