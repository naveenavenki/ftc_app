package org.firstinspires.ftc.teamcode;

/**
 * Created by shiv on 10/14/2016.
 */
public interface BokAutoTest {
    static final float LEFT_MOTOR_POWER = 0.5f;
    static final float RIGHT_MOTOR_POWER = 0.525f;

    public void initTest(BoKAuto opMode, BoKHardwareBot robot);
    public void runTest(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException;
}
