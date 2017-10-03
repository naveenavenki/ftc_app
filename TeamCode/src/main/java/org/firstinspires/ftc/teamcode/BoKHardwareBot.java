package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public abstract class BoKHardwareBot
{
    // Constants
    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT  = 10;

    // DC motors

    // Servos

    // Sensors

    // waitForTicks
    private ElapsedTime period  = new ElapsedTime();

    // return status
    public enum BoKHardwareStatus
    {
        BOK_HARDWARE_FAILURE,
        BOK_HARDWARE_SUCCESS
    }

    /*
     * The initHardware() method initializes the hardware on the robot including the drive train.
     * It calls the abstract initDriveTrainMotors() and initMotorsAndSensors() methods.
     * Returns BOK_SUCCESS if the initialization is successful, BOK_FAILURE otherwise.
     */
    public BoKHardwareStatus initHardware(LinearOpMode opMode)
    {
        // First initialize the drive train
        BoKHardwareStatus rc = initDriveTrainMotors(opMode);
        if (rc == BoKHardwareStatus.BOK_HARDWARE_SUCCESS) {
            // Next initialize the other motors and sensors
            rc = initMotorsAndSensors(opMode);
        }
        return rc;
    }

    /*
     * The initMotorsAndSensors() method initializes the motors (other than the drive train) and
     * sensors on the robot.
     */
    private BoKHardwareStatus initMotorsAndSensors(LinearOpMode opMode)
    {
        return BoKHardwareStatus.BOK_HARDWARE_SUCCESS;
    }

    // Initialization of drive train is protected but abstract
    protected abstract BoKHardwareStatus initDriveTrainMotors(LinearOpMode opMode);

    // Using the drive train is public
    public abstract void setModeForDTMotors(DcMotor.RunMode runMode);
    //public abstract void setPowerToDTMotors(double leftPower, double rightPower);
    public abstract void setPowerToDTMotors(double leftFrontPower,
                                            double leftBackPower,
                                            double rightFrontPower,
                                            double rightBackPower);

    // Autonomous driving
    public abstract void move(double leftPower,
                              double rightPower,
                              double inches,
                              boolean forward);

    public abstract boolean getDTCurrentPosition();

    /*
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
