package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public abstract class BoKHardwareBot {
    // Constants
    private static final String COLOR_SENSOR_CFG        = "color";
    private static final String GYRO_SENSOR_CFG         = "gy";
    private static final String ODS_SENSOR_CFG          = "ods";
    private static final String RANGE_SENSOR_CFG        = "rs";
    private static final String SERVO_SHOOTER_CFG       = "ss";
    private static final String SERVO_PUSHER_LEFT_CFG   = "pl";
    private static final String SERVO_PUSHER_RIGHT_CFG  = "pr";
    private static final String MOTOR_LEFT_SHOOTER_CFG  = "sl";
    private static final String MOTOR_RIGHT_SHOOTER_CFG = "sr";
    private static final String MOTOR_SWEEPER_CFG       = "sw";

    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT  = 10;
    protected static final int OPMODE_SLEEP_INTERVAL_MS_LONG   = 100;

    // LinearOpMode
    LinearOpMode currentOpMode;

    // Sensors
    protected ColorSensor colorSensor;
    protected ModernRoboticsI2cGyro gyroSensor;
    protected OpticalDistanceSensor odsSensor;
    protected ModernRoboticsI2cRangeSensor rangeSensor;


    //servos: shooter and button pushers
    protected Servo shooterServo;
    protected Servo pusherLeftServo;
    protected Servo pusherRightServo;

    protected static final double INITIAL_SHOOTER_SERVO_POS_TELEOP    = 0.07;
    protected static final double INITIAL_SHOOTER_SERVO_POS_AUTO      = 0.07;
    protected static final double INITIAL_SERVO_POS_PUSHER_LEFT  = 0.04;
    protected static final double FINAL_SERVO_POS_PUSHER_LEFT    = 0.5;
    protected static final double INITIAL_SERVO_POS_PUSHER_RIGHT = 0.95;
    protected static final double FINAL_SERVO_POS_PUSHER_RIGHT   = 0.5;

    //shooter motors and sweeper motor
    private DcMotor leftShooterMotor;
    private DcMotor rightShooterMotor;
    protected DcMotor sweeperMotor;

    protected static final double SHOOTER_MOTORS_POWER = 0.8;
    protected static final double SHOOTER_MOTORS_POWER_TELEOP = 1.0;
    protected static final double SWEEPER_MOTOR_POWER_NORMAL  = 1.0;//0.95
    protected static final double SWEEPER_MOTOR_POWER_REVERSE = -0.5;

    // waitForTicks
    private ElapsedTime period  = new ElapsedTime();

    // return status
    public enum BoKStatus {
        BOK_FAILURE,
        BOK_SUCCESS
    }

    public BoKStatus initHardware(LinearOpMode opMode) {
        // first initialize the drive train
        BoKStatus rc = initMotors(opMode);
        if (rc == BoKStatus.BOK_SUCCESS) {
            rc = initMotorsAndSensors(opMode);
        }
        return rc;
    }

    /*
     * Initialize the sensor variables.
     * The initSensors() method of the hardware class does all the work here
     */
    private BoKStatus initMotorsAndSensors(LinearOpMode opMode) {
        /*
        colorSensor = opMode.hardwareMap.colorSensor.get(COLOR_SENSOR_CFG);
        if (colorSensor == null) {
            return BoKStatus.BOK_FAILURE;
        }
        */
        gyroSensor = (ModernRoboticsI2cGyro)opMode.hardwareMap.gyroSensor.get(GYRO_SENSOR_CFG);
        if (gyroSensor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        odsSensor = opMode.hardwareMap.opticalDistanceSensor.get(ODS_SENSOR_CFG);
        if (odsSensor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        rangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGE_SENSOR_CFG);
        if (rangeSensor == null) {
            return BoKStatus.BOK_FAILURE;
        }
        shooterServo = opMode.hardwareMap.servo.get(SERVO_SHOOTER_CFG);
        if (shooterServo == null) {
          return BoKStatus.BOK_FAILURE;
        }

        pusherLeftServo = opMode.hardwareMap.servo.get(SERVO_PUSHER_LEFT_CFG);
        if (pusherLeftServo == null) {
            return BoKStatus.BOK_FAILURE;
        }
        pusherRightServo = opMode.hardwareMap.servo.get(SERVO_PUSHER_RIGHT_CFG);
        if (pusherRightServo == null) {
            return BoKStatus.BOK_FAILURE;
        }

        leftShooterMotor = opMode.hardwareMap.dcMotor.get(MOTOR_LEFT_SHOOTER_CFG);
        if (leftShooterMotor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        rightShooterMotor = opMode.hardwareMap.dcMotor.get(MOTOR_RIGHT_SHOOTER_CFG);
        if (rightShooterMotor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        sweeperMotor = opMode.hardwareMap.dcMotor.get(MOTOR_SWEEPER_CFG);
        if (sweeperMotor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);      // start the sweeper motor for ball intake
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE); // reverse the right ball shooter

        return BoKStatus.BOK_SUCCESS;
    }

    // Initialization of drive train is protected
    protected abstract BoKStatus initMotors(LinearOpMode opMode);

    // Using the drive train is public
    public abstract void setModeForMotors(DcMotor.RunMode runMode);
    public abstract void setPowerToMotors(double leftPower, double rightPower);

    // RUN_USING_ENCODER
    //public abstract void setupMotorEncoders(LinearOpMode opMode);
    public abstract void setMotorEncoderTarget(int leftTarget, int rightTarget);

    public abstract boolean getCurrentPosition();

    // Using the shooter motors (we want to control both motors at the same time)
    public void setPowerToShooter(double shooterPow){
        leftShooterMotor.setPower(shooterPow);
        rightShooterMotor.setPower(shooterPow);
    }

    public void initGyro(LinearOpMode opMode) {
        Log.v("BOK", "Calibrating gyro");
        gyroSensor.calibrate();
        // make sure the gyro is calibrated before continuing
        while (!opMode.isStopRequested() && gyroSensor.isCalibrating())  {
            opMode.sleep(50);
            opMode.idle();
        }

        opMode.sleep(250);
        gyroSensor.resetZAxisIntegrator();
    }

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
