package org.firstinspires.ftc.teamcode.threadedhardware;

public class NullThread implements ActionThread {

    //Class implementation of DThread

    public NullThread(){}

    public void Stop() {}

    public void start() {}

    public boolean isAlive() {
        return false;
    }

    @Override
    public void run() {
        //Do nothing
    }
}
