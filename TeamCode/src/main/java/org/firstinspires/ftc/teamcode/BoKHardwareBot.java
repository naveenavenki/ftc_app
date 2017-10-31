package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public abstract class BoKHardwareBot
{
    // Constants
    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT  = 10;

    protected static final double CW_INIT = 0.85;
    protected static final double CW_MID = 0.45;
    protected static final double CG_INIT = 0.92;
    protected static final double CG_MID = 0.5;
    protected static final double CG_CLOSE = 0.92;
    protected static final double JF_INIT = 0.0;
    protected static final double JF_FINAL = 0.5;
    protected static final double JF_HIT_CRYPTO = 0.4;
    protected static final double JA_INIT = 0.04;
    protected static final double JA_MID = 0.4;
    protected static final double JA_FINAL = 0.5;

    protected static final double RA_INIT = 0.75;



    private static final String TURN_TABLE_MOTOR = "tt";
    private static final String UPPER_ARM_MOTOR  = "ua";
    private static final String CLAW_WRIST_SERVO = "cw";
    private static final String CLAW_GRAB_SERVO  = "cg";
    private static final String JEWEL_ARM  = "ja";
    private static final String JEWEL_FLICKER  = "jf";
    private static final String RELIC_ARM_SERVO  = "ra";


    // DC motors
    protected DcMotor turnTable;
    protected DcMotor upperArm;
    protected DcMotor spool;

    // Servos

    private Servo clawWrist; // These are used in the GlyphArm
    private Servo clawGrab;
    protected Servo jewelArm;
    protected Servo jewelFlicker;
    protected Servo relicArm;

    // Sensors
    BNO055IMU imu;
    DigitalChannel flickerTouch;

    // Glyph Arm
    BoKGlyphArm glyphArm;

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
        clawWrist = opMode.hardwareMap.servo.get(CLAW_WRIST_SERVO);
        if(clawWrist == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        clawGrab = opMode.hardwareMap.servo.get(CLAW_GRAB_SERVO);
        if(clawGrab == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        jewelArm = opMode.hardwareMap.servo.get(JEWEL_ARM);
        if(jewelArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        jewelFlicker = opMode.hardwareMap.servo.get(JEWEL_FLICKER);
        if(jewelFlicker == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicArm = opMode.hardwareMap.servo.get(RELIC_ARM_SERVO);
        if(relicArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        spool = opMode.hardwareMap.dcMotor.get("sp");
        if(spool == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        upperArm = opMode.hardwareMap.dcMotor.get(UPPER_ARM_MOTOR);
        if(upperArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        turnTable = opMode.hardwareMap.dcMotor.get(TURN_TABLE_MOTOR);
        if(turnTable == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        flickerTouch = opMode.hardwareMap.get(DigitalChannel.class, "ft");
        if(flickerTouch == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        clawWrist.setPosition(CW_INIT);
        clawGrab.setPosition(CG_INIT);
        jewelArm.setPosition(JA_INIT);
        jewelFlicker.setPosition(JF_INIT);
        relicArm.setPosition(RA_INIT);

        upperArm.setDirection(DcMotorSimple.Direction.REVERSE);
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //upperArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTable.setPower(0);
        turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flickerTouch.setMode(DigitalChannel.Mode.INPUT);

        glyphArm = new BoKGlyphArm(this, opMode, clawWrist, clawGrab);
        return BoKHardwareStatus.BOK_HARDWARE_SUCCESS;
    }

    // Initialization of drive train is protected but abstract
    protected abstract BoKHardwareStatus initDriveTrainMotors(LinearOpMode opMode);

    // Using the drive train is public
    public abstract void resetDTEncoders();
    public abstract boolean areDTMotorsBusy();

    //public abstract void setPowerToDTMotors(double leftPower, double rightPower);
    public abstract void setPowerToDTMotors(double leftFrontPower,
                                            double leftBackPower,
                                            double rightFrontPower,
                                            double rightBackPower);

    // Autonomous driving
    public abstract void startMove(double leftPower,
                                   double rightPower,
                                   double inches,
                                   boolean backward);

    public abstract void startStrafe(double power, double rotations,
                                     boolean right);

    public abstract void stopMove();

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
