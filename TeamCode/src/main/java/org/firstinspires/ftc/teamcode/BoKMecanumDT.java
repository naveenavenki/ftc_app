package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Krishna Saxena on 10/2/2017.
 * Extends BoKHardwareBot to implement the Mecanum wheels drive train with 4 DC Motors.
 */
public class BoKMecanumDT extends BoKHardwareBot
{
    // CONSTANTS
    // 280 cycles per revolution (CPR); It is a quadrature encoder producing 4 Pulses per Cycle.
    // With 280 CPR, it outputs 1120 PPR. AndyMark 40 Motor Encoder
    // For 360 degrees wheel turn, motor shaft moves 480 degrees (approx)
    private static final double   COUNTS_PER_MOTOR_REV    = 1120;
    private static final double   DRIVE_GEAR_REDUCTION    = 1.33;
    private static final double   WHEEL_DIAMETER_INCHES   = 4.0;
    private static final int DISTANCE_THRESHOLD           = 10;

    // CONSTANTS (strings from the robot config)
    private static final String LEFT_BACK_MOTOR_NAME   = "lb";
    private static final String LEFT_FRONT_MOTOR_NAME  = "lf";
    private static final String RIGHT_BACK_MOTOR_NAME  = "rb";
    private static final String RIGHT_FRONT_MOTOR_NAME = "rf";

    // Drive train motors
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    // Encoder variables
    private int currentLeftTarget;
    private int currentRightTarget;
    private boolean leftPositive;
    private boolean rightPositive;
    private boolean leftReached;
    private boolean rightReached;

    LinearOpMode opMode; // current opMode

    /*
     * Implement all the abstract methods
     * Initialize the drive system variables.
     * The initDriveTrainMotors() method of the hardware class does all the work here
     */
    protected BoKHardwareStatus initDriveTrainMotors(LinearOpMode opMode)
    {
        this.opMode = opMode;

        leftBack = opMode.hardwareMap.dcMotor.get(LEFT_BACK_MOTOR_NAME);
        if (leftBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        leftFront = opMode.hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR_NAME);
        if (leftFront == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rightBack = opMode.hardwareMap.dcMotor.get(RIGHT_BACK_MOTOR_NAME);
        if (rightBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rightFront = opMode.hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR_NAME);
        if (rightFront == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Drive train is initialized, initialize sensors
        return BoKHardwareStatus.BOK_HARDWARE_SUCCESS;
    }

    /*
     * Set methods:
     * 1. set power
     * 2. set mode
     * 3. set motor encoder target
     */
    public void setPowerToDTMotors(double left, double right)
    {
        leftBack.setPower(left);
        rightBack.setPower(right);
        leftFront.setPower(left);
        rightFront.setPower(right);
        opMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    public void setPowerToDTMotors(double leftFrontPower, double leftBackPower,
                                   double rightFrontPower, double rightBackPower)
    {
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        opMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    public void setModeForDTMotors(DcMotor.RunMode runMode)
    {
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        opMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    /*
     * getTargetEncCount(targetDistanceInches): returns the target encoder count
     * based on the wheel diameter, gear reduction ratio and counts per motor rev.
     */
    public double getTargetEncCount(double targetDistanceInches)
    {
        double degreesOfWheelTurn, degreesOfMotorTurn;
        degreesOfWheelTurn = (360.0 / (Math.PI * BoKMecanumDT.WHEEL_DIAMETER_INCHES)) *
                targetDistanceInches;
        degreesOfMotorTurn = BoKMecanumDT.DRIVE_GEAR_REDUCTION * degreesOfWheelTurn;
        return (BoKMecanumDT.COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;
    }

    private void setDTMotorEncoderTarget(int leftTarget, int rightTarget)
    {
        // We are NOT using RUN_USING_ENCODER or RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftReached = rightReached = false;

        currentLeftTarget = leftFront.getCurrentPosition() + leftTarget;
        currentRightTarget = rightFront.getCurrentPosition() + rightTarget;

        opMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
        //opMode.idle();

        if (leftTarget > 0)
            leftPositive = true;
        else
            leftPositive = false;

        if (rightTarget > 0)
            rightPositive = true;
        else
            rightPositive = false;

        Log.v("BOK", "START: " + leftFront.getCurrentPosition() +", " + currentLeftTarget + ", " +
                rightFront.getCurrentPosition() + ", " + currentRightTarget);
    }

    /*
     * move() method: setup the robot to move encoder counts
     */
    public void move(double leftPower,
                     double rightPower,
                     double inches,
                     boolean forward)
    {
        double targetEncCount = getTargetEncCount(inches);
        if (forward) {
            setDTMotorEncoderTarget((int) -targetEncCount, (int) targetEncCount);
            setPowerToDTMotors(leftPower, leftPower, -rightPower, -rightPower);
        }
        else {
            setDTMotorEncoderTarget((int) targetEncCount, (int) -targetEncCount);
            setPowerToDTMotors(-leftPower, -leftPower, rightPower, rightPower);
        }
    }

    /*
     * getDTCurrentPosition() method: Returns true if target encoder count is reached
     */
    public boolean getDTCurrentPosition()
    {
        int leftFrontCurrentPos = leftFront.getCurrentPosition();
        int rightFrontCurrentPos = rightFront.getCurrentPosition();

        //Log.v("BOK", "Current " + leftFrontCurrentPos + ", " + leftFront.isBusy() + ", " +
        //             rightFrontCurrentPos + ", " + rightFront.isBusy());

        if (leftPositive) {
            if ((leftFrontCurrentPos >= currentLeftTarget) ||
                    (Math.abs(leftFrontCurrentPos - currentLeftTarget) <= DISTANCE_THRESHOLD)) {
                Log.v("BOK", "Left position reached +ve!!");
                leftReached = true;
            }
        }
        else {
            if ((leftFrontCurrentPos <= currentLeftTarget) ||
                    (Math.abs(leftFrontCurrentPos - currentLeftTarget) <= DISTANCE_THRESHOLD)) {
                Log.v("BOK", "Left position reached -ve!!");
                leftReached = true;
            }
        }

        if (rightPositive) {
            if ((rightFrontCurrentPos >= currentRightTarget) ||
                    (Math.abs(rightFrontCurrentPos - currentRightTarget) <= DISTANCE_THRESHOLD)) {
                Log.v("BOK", "Right position reached +ve!!");
                rightReached = true;
            }
        }
        else {
            if ((rightFrontCurrentPos <= currentRightTarget) ||
                    (Math.abs(rightFrontCurrentPos - currentRightTarget) <= DISTANCE_THRESHOLD)) {
                Log.v("BOK", "Right position reached -ve!!");
                rightReached = true;
            }
        }

        return rightReached && leftReached;
    }
}