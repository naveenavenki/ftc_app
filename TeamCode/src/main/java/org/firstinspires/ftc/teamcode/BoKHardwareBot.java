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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Krishna Saxena on 9/24/2016.
 * This class implements all of the non drive train functionality. It deals with initializing
 * the servos, the non drive train DC motors, and the sensors.
 */
public abstract class BoKHardwareBot {

    // Constants strings from the robot config
    //private static final String COLOR_SENSOR_CFG        = "color";
    private static final String GYRO_SENSOR_CFG         = "gy";

    private static final String ODS_SENSOR_CFG          = "ods";
    private static final String RANGE_SENSOR_LEFT_CFG   = "rsl";
    private static final String RANGE_SENSOR_RIGHT_CFG  = "rsr";
    private static final String SERVO_SHOOTER_CFG       = "ss";
    private static final String SERVO_PUSHER_LEFT_CFG   = "pl";
    private static final String SERVO_PUSHER_RIGHT_CFG  = "pr";
    private static final String MOTOR_LEFT_SHOOTER_CFG  = "sl";
    private static final String MOTOR_RIGHT_SHOOTER_CFG = "sr";
    private static final String MOTOR_SWEEPER_CFG       = "sw";
    private static final String MOTOR_CAP_LIFT_CFG      = "lift";
    private static final String SERVO_CAP_LIFT_CFG      = "lifts";
    private static final String SERVO_PART_LIFT_GATE_CFG= "gate";
    private static final String MOTOR_CTRL_UP_RIGHT_CFG = "Motor Controller Up Right";

    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT  = 10;
    protected static final int OPMODE_SLEEP_INTERVAL_MS_LONG   = 100;

    // LinearOpMode
    LinearOpMode currentOpMode;

    // Sensors
    protected ColorSensor colorSensor;
    protected ModernRoboticsI2cGyro gyroSensor;
    protected OpticalDistanceSensor odsSensor;
    protected I2cDevice rangeSensorLeft;
    protected I2cDevice rangeSensorRight;
    protected I2cDeviceSynch rsLeftReader;
    protected I2cDeviceSynch rsRightReader;

    //servos: shooter, button pushers, cap claw lock, part lift gate
    protected Servo shooterServo;
    protected Servo pusherLeftServo;
    protected Servo pusherRightServo;
    //protected Servo clawLockServo;
    //protected Servo partLiftGateServo;
    protected static final double INITIAL_SHOOTER_SERVO_POS_TELEOP    = 0.08;
    protected static final double INITIAL_SHOOTER_SERVO_POS_AUTO      = 0.06;
    protected static final double INITIAL_SERVO_POS_PUSHER_LEFT  = 0.85;
    protected static final double FINAL_SERVO_POS_PUSHER_LEFT    = 0.35;
    protected static final double INITIAL_SERVO_POS_PUSHER_RIGHT = 0.3;
    protected static final double FINAL_SERVO_POS_PUSHER_RIGHT   = 0.8;
    protected static final double INITIAL_SERVO_POS_CAP_CLAW     = 0.9;
    protected static final double FINAL_SERVO_POS_CAP_CLAW       = 0.5;
    protected static final double INITIAL_SERVO_POS_PART_GATE    = 0.0;
    protected static final double FINAL_SERVO_POS_PART_GATE      = 0.65;
    protected static final double INITIAL_SERVO_POS_CAP_BALL     = 0.9;
    protected static final double FINAL_SERVO_POS_CAP_BALL       = 0.1;



    //shooter motors, sweeper motor, and cap ball lift motor
    private DcMotor leftShooterMotor;
    private DcMotor rightShooterMotor;
    protected DcMotor sweeperMotor;
   // protected DcMotor capLiftMotor;

    protected static final double SHOOTER_MOTORS_POWER_NORMAL = 0.8;
    protected static final double SWEEPER_MOTOR_POWER_NORMAL  = 0.95;
    protected static final double SWEEPER_MOTOR_POWER_REVERSE = -0.5;

    protected VoltageSensor voltageSensor;

    // waitForTicks
    private ElapsedTime period  = new ElapsedTime();

    // return status
    public enum BoKStatus {
        BOK_FAILURE,
        BOK_SUCCESS
    }

    /*
     * Drive Train support for the robot. The robot can use any drive train as long as
     * the following methods are defined. Note that the methods are declared abstract here!
     */

    // Initialization of drive train is protected
    protected abstract BoKStatus initDriveTrainMotors(LinearOpMode opMode);

    // Using the drive train (DT) is public (teleop opMode)
    public abstract void setModeForDTMotors(DcMotor.RunMode runMode);
    public abstract void setPowerToDTMotors(double leftPower, double rightPower);
    public abstract void setPowerToDTMotors(double leftFrontPower, double leftBackPower,
                                            double rightFrontPower, double rightBackPower);

    // Setting up the drive train motors encoders (autonomous opMode)
    public abstract void setDTMotorEncoderTarget(int leftTarget, int rightTarget);

    // Return true if the drive train has reached its target, false otherwise
    public abstract boolean getDTCurrentPosition();

    public BoKStatus initHardware(LinearOpMode opMode) {
        // first initialize the drive train
        BoKStatus rc = initDriveTrainMotors(opMode);
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

        rangeSensorLeft = opMode.hardwareMap.i2cDevice.get(RANGE_SENSOR_LEFT_CFG);
        if (rangeSensorLeft == null) {
            return BoKStatus.BOK_FAILURE;
        }
        rangeSensorRight = opMode.hardwareMap.i2cDevice.get(RANGE_SENSOR_RIGHT_CFG);
        if (rangeSensorRight == null) {
            return BoKStatus.BOK_FAILURE;
        }

        rsLeftReader = new I2cDeviceSynchImpl(rangeSensorLeft, I2cAddr.create8bit(0x2a), false);
        rsRightReader = new I2cDeviceSynchImpl(rangeSensorRight, I2cAddr.create8bit(0x28), false);
        rsLeftReader.engage();
        rsRightReader.engage();

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
/*
        capLiftMotor = opMode.hardwareMap.dcMotor.get(MOTOR_CAP_LIFT_CFG);
        if (capLiftMotor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        clawLockServo = opMode.hardwareMap.servo.get(SERVO_CAP_LIFT_CFG);
        if (clawLockServo == null) {
            return BoKStatus.BOK_FAILURE;
        }

        partLiftGateServo = opMode.hardwareMap.servo.get(SERVO_PART_LIFT_GATE_CFG);
        if (partLiftGateServo == null) {
            return BoKStatus.BOK_FAILURE;
        }
*/
        voltageSensor = opMode.hardwareMap.voltageSensor.get(MOTOR_CTRL_UP_RIGHT_CFG);
        if (voltageSensor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        // set up the robot attachment motors
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // start the sweeper motor for particle intake
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // reverse the right particle shooter motor
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //capLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return BoKStatus.BOK_SUCCESS;
    }

    // Using the shooter motors (we want to control both motors at the same time)
    public void setPowerToShooterMotors(double shooterPow){
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
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
