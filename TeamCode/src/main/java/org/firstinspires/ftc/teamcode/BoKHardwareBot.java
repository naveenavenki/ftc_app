package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.util.Log;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public abstract class BoKHardwareBot
{
    // Constants
    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT  = 10;

    protected static final double CW_INIT = 0.92;
    protected static final double CW_MIN = 0.1;
    protected static final double CW_MID = 0.45;
    protected static final double CG_INIT = 0.8; // Closed at initialization
    protected static final double CG_OPEN = 0.4;
    protected static final double CG_CLOSE = 0.8;
    protected static final double JF_INIT = 0.8;
    protected static final double JF_FINAL = 0.42;
    protected static final double JF_RIGHT = 1;
    protected static final double JF_LEFT = 0;
    protected static final double JA_INIT = 0.02;
    protected static final double JA_MID = 0.42;
    protected static final double JA_FINAL = 0.46;

    protected static final double RA_INIT = 0.11;
    protected static final double RELIC_ARM_UPPER_LIMIT = 0.18;
    protected static final double SP_INIT = 0.95;
    protected static final double RC_UNLOCK = 1; // initially unlocked
    protected static final double RC_LOCK = 0.4;
    protected static final double RL_INIT = 0.2;
    protected static final double RL_UNLOCK = 0;

    private static final String TURN_TABLE_MOTOR = "tt";
    private static final String UPPER_ARM_MOTOR  = "ua";
    private static final String RELIC_SPOOL_SERVO = "sp";
    private static final String CLAW_WRIST_SERVO = "cw";
    private static final String CLAW_GRAB_SERVO  = "cg";
    private static final String JEWEL_ARM_SERVO  = "ja";
    private static final String JEWEL_FLICKER_SERVO  = "jf";
    private static final String RELIC_ARM_SERVO = "ra";
    private static final String RELIC_CLAW_SERVO = "rc";
    private static final String RELIC_LOCK_SERVO = "rl";
    //private static final String RIGHT_GLYPH_CLAW_SERVO  = "rg";
    //private static final String LEFT_GLYPH_CLAW_SERVO  = "lg";
    private static final String RANGE_SENSOR_JA = "rs";
    private static final String RANGE_SENSOR_FRONT_CFG  = "rsf";
    private static final String RANGE_SENSOR_BACK_CFG   = "rsb";
    private static final String IMU_TOP = "imu_top";

    protected static final int WAIT_PERIOD = 40; // 40 ms

    // DC motors
    protected DcMotor turnTable;
    protected DcMotor upperArm;

    // Servos
    private Servo glyphClawWrist; // These are used in the GlyphArm
    private Servo glyphClawGrab;
    protected Servo jewelArm;
    protected Servo jewelFlicker;
    protected Servo relicArm;
    protected Servo relicSpool;
    protected Servo relicClaw;
    protected Servo relicLock;

    protected CRServo rightGlyphServo;
    protected CRServo leftGlyphServo;

    // Sensors
    protected BNO055IMU imu;

    protected ModernRoboticsI2cRangeSensor rangeSensorJA;
    protected ModernRoboticsI2cRangeSensor rangeSensorFront;
    protected ModernRoboticsI2cRangeSensor rangeSensorBack;

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
/*
        rightGlyphServo = opMode.hardwareMap.crservo.get(RIGHT_GLYPH_CLAW_SERVO);
        if(rightGlyphServo == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        leftGlyphServo = opMode.hardwareMap.crservo.get(LEFT_GLYPH_CLAW_SERVO);
        if(leftGlyphServo == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
*/
        glyphClawWrist = opMode.hardwareMap.servo.get(CLAW_WRIST_SERVO);
        if(glyphClawWrist == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        glyphClawGrab = opMode.hardwareMap.servo.get(CLAW_GRAB_SERVO);
        if(glyphClawGrab == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        jewelArm = opMode.hardwareMap.servo.get(JEWEL_ARM_SERVO);
        if(jewelArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        jewelFlicker = opMode.hardwareMap.servo.get(JEWEL_FLICKER_SERVO);
        if(jewelFlicker == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicArm = opMode.hardwareMap.servo.get(RELIC_ARM_SERVO);
        if(relicArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicSpool = opMode.hardwareMap.servo.get(RELIC_SPOOL_SERVO);
        if(relicSpool == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicClaw = opMode.hardwareMap.servo.get(RELIC_CLAW_SERVO);
        if(relicClaw == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicLock = opMode.hardwareMap.servo.get(RELIC_LOCK_SERVO);
        if(relicLock == null){
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

        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_TOP);
        if(imu == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rangeSensorJA = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,
                RANGE_SENSOR_JA);
        if (rangeSensorJA == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rangeSensorFront = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,
                RANGE_SENSOR_FRONT_CFG);
        if (rangeSensorFront == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rangeSensorBack = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,
            RANGE_SENSOR_BACK_CFG);
        if (rangeSensorBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        if (!opMode.getClass().getName().contains("Tele")) {
            //Make sure that the flicker is open
            jewelFlicker.setPosition(JF_FINAL);


            File file = AppUtil.getInstance().getSettingsFile("BoKArmCalibration.txt");
            String value = ReadWriteFile.readFile(file);
            if (value.isEmpty()) {
                Log.v("BOK", "File not found");
                glyphClawWrist.setPosition(CW_INIT);
            } else {
                Log.v("BOK", "Calibration data: " + value);
                glyphClawWrist.setPosition(Double.parseDouble(value));
            }
        }

        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnTable.setPower(0);
        //turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       // right.setDirection(CRServo.Direction.REVERSE);

        if (!opMode.getClass().getName().contains("Tele")) {
            glyphClawGrab.setPosition(CG_INIT);
            jewelArm.setPosition(JA_INIT);
            jewelFlicker.setPosition(JF_INIT);
            relicArm.setPosition(RA_INIT);
            relicSpool.setPosition(SP_INIT);
            relicClaw.setPosition(RC_UNLOCK);
            relicLock.setPosition(RL_INIT);
        }
        else {
            //glyphClawWrist.setPosition(CW_INIT);
            //glyphClawGrab.setPosition(CG_INIT);
            //jewelArm.setPosition(JA_INIT);
            //jewelFlicker.setPosition(JF_INIT);
            relicArm.setPosition(RA_INIT);
            relicSpool.setPosition(SP_INIT);
            relicClaw.setPosition(RC_UNLOCK);
            relicLock.setPosition(RL_INIT);
        }

        upperArm.setDirection(DcMotorSimple.Direction.REVERSE);
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //upperArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperArm.setPower(0);

        glyphArm = new BoKGlyphArm(this, opMode, glyphClawWrist, glyphClawGrab);
       
        return BoKHardwareStatus.BOK_HARDWARE_SUCCESS;
    }

    protected void initializeImu()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //angles = new Orientation();
        imu.initialize(parameters);
    }

    // Initialization of drive train is protected but abstract
    protected abstract BoKHardwareStatus initDriveTrainMotors(LinearOpMode opMode);

    // Using the drive train is public
    public abstract void resetDTEncoders();
    public abstract boolean areDTMotorsBusy();
    public abstract boolean haveDTMotorsReachedTarget();

    //public abstract void setPowerToDTMotors(double leftPower, double rightPower);
    public abstract void setPowerToDTMotors(double leftFrontPower,
                                            double leftBackPower,
                                            double rightFrontPower,
                                            double rightBackPower);
    public abstract void setModeForDTMotors(DcMotor.RunMode runMode);

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
    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            }
            catch (InterruptedException e) {
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
