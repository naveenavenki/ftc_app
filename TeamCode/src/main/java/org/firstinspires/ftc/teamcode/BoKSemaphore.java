package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 1/17/2018.
 */

public class BoKSemaphore {
    private boolean signal = false;

    public synchronized void take() {
        this.signal = true;
        this.notify();
    }

    public synchronized void release() throws InterruptedException {
        while(!this.signal) wait();
        this.signal = false;
    }
}