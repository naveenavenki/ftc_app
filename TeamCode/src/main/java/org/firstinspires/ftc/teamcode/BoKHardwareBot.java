package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public abstract class BoKHardwareBot {
    // Constants
    private static final String COLOR_SENSOR_NAME = "color";

    // Sensors
    public ColorSensor colorSensor;

    // waitForTicks
    private ElapsedTime period  = new ElapsedTime();

    // return status
    public enum BoKStatus {
        BOK_FAILURE,
        BOK_SUCCESS
    }

    public BoKStatus initHardware(OpMode opMode) {
        // first initialize the drive train
        BoKStatus rc = initMotors(opMode);
        if (rc == BoKStatus.BOK_SUCCESS) {
            rc = initSensors(opMode);
        }
        return rc;
    }

    /*
     * Initialize the sensor variables.
     * The initSensors() method of the hardware class does all the work here
     */
    private BoKStatus initSensors(OpMode opMode) {
        colorSensor = opMode.hardwareMap.colorSensor.get(COLOR_SENSOR_NAME);
        if (colorSensor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        return BoKStatus.BOK_SUCCESS;
    }

    // Initialization of drive train is protected
    protected abstract BoKStatus initMotors(OpMode opMode);

    // Using the drive train is public
    public abstract void setModeForMotors(DcMotor.RunMode runMode);
    public abstract void setPowerToMotors(double leftPower, double rightPower);

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
