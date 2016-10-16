package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by shiv on 10/14/2016.
 */
public class BoKAutoRunTest implements BokAutoTest {
    @Override
    public void initTest(BoKAuto opMode, BoKHardwareBot robot) {
    }

    @Override
    public void runTest(BoKAuto opMode, BoKHardwareBot robot) throws InterruptedException {
        if (opMode.opModeIsActive()) {
            robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerToMotors(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);

            // run until the end of the match (driver presses STOP)
            while (opMode.opModeIsActive()) {
                // Allow time for other processes to run.
                opMode.idle();
            } // while (opModeIsActive())
        } // if (opModeIsActive())

    }
}
