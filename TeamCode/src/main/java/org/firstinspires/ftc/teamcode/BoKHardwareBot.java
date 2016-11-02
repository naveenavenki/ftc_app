package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public abstract class BoKHardwareBot {
    // Constants
    private static final String COLOR_SENSOR_CFG        = "color";
    private static final String SERVO_SHOOTER_CFG       = "ss";
    private static final String SERVO_PUSHER_LEFT_CFG   = "pl";
    private static final String SERVO_PUSHER_RIGHT_CFG  = "pr";
    private static final String MOTOR_LEFT_SHOOTER_CFG  = "sl";
    private static final String MOTOR_RIGHT_SHOOTER_CFG = "sr";
    private static final String MOTOR_SWEEPER_CFG       = "sw";

    // Sensors
    public ColorSensor colorSensor;

    //servos
    //public Servo shooterServo;
    protected Servo pusherLeftServo;
    protected Servo pusherRightServo;

    //shooter and sweeper
    protected DcMotor leftShooterMotor;
    protected DcMotor rightShooterMotor;
    protected DcMotor sweeperMotor;

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
            rc = initMotorsAndSensors(opMode);
        }
        return rc;
    }

    /*
     * Initialize the sensor variables.
     * The initSensors() method of the hardware class does all the work here
     */
    private BoKStatus initMotorsAndSensors(OpMode opMode) {
        colorSensor = opMode.hardwareMap.colorSensor.get(COLOR_SENSOR_CFG);
        if (colorSensor == null) {
            return BoKStatus.BOK_FAILURE;
        }

        //shooterServo = opMode.hardwareMap.servo.get(SERVO_SHOOTER_CFG);
        //if (shooterServo == null) {
          //  return BoKStatus.BOK_FAILURE;
        //}

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

        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE); // reverse the right ball shooter

        return BoKStatus.BOK_SUCCESS;
    }

    // Initialization of drive train is protected
    protected abstract BoKStatus initMotors(OpMode opMode);

    // Using the drive train is public
    public abstract void setModeForMotors(DcMotor.RunMode runMode);
    public abstract void setPowerToMotors(double leftPower, double rightPower);

    // Using the sweeper motor
    public void setPowerToSweeper(double sweeperPow){
        sweeperMotor.setPower(sweeperPow);

    }
    // Using the shooter motors
    public void setPowerToShooter(double shooterPow){
        leftShooterMotor.setPower(shooterPow);
        rightShooterMotor.setPower(shooterPow);
    }

    public void setLeftPusherPos(double leftPos)
    {
        pusherLeftServo.setPosition(leftPos);
    }

    public void setRightPusherPos(double rightPos)
    {
        pusherRightServo.setPosition(rightPos);
    }
/*
    public void setShooterServoPos(double pos){
        shooterServo.setPosition(pos);
    }
*/
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
