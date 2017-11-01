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
    // 134.4 cycles per revolution (CPR); It is a quadrature encoder producing 4 Pulses per Cycle.
    // With 134.4 CPR, it outputs 537.6 PPR. AndyMark Orbital 20 Motor Encoder
    // For 360 degrees wheel turn, motor shaft moves 480 degrees (approx)
    private static final double   COUNTS_PER_MOTOR_REV    = 537.6;
    private static final double   DRIVE_GEAR_REDUCTION    = 1.33;
    private static final double   WHEEL_DIAMETER_INCHES   = 4.0;

    // CONSTANTS (strings from the robot config)
    private static final String LEFT_BACK_MOTOR_NAME   = "lb";
    private static final String LEFT_FRONT_MOTOR_NAME  = "lf";
    private static final String RIGHT_BACK_MOTOR_NAME  = "rb";
    private static final String RIGHT_FRONT_MOTOR_NAME = "rf";

    // Drive train motors
    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor rightFront;

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

        setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void setZeroPowerBehaviorDTMotors() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void resetDTEncoders()
    {
        // all four motors need encoder wires to use RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDTMotorEncoderTarget(int leftTarget, int rightTarget)
    {
        int currentLeftTarget = leftFront.getCurrentPosition() + leftTarget;
        int currentRightTarget = rightFront.getCurrentPosition() + rightTarget;

        leftFront.setTargetPosition(currentLeftTarget);
        leftBack.setTargetPosition(currentLeftTarget);
        rightFront.setTargetPosition(currentRightTarget);
        rightBack.setTargetPosition(currentRightTarget);

        // Turn On RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_TO_POSITION);

        Log.v("BOK", "START: " + leftFront.getCurrentPosition() +", " + currentLeftTarget + ", " +
                rightFront.getCurrentPosition() + ", " + currentRightTarget);
    }
    
    private void setDTMotorEncoderTarget(int leftFrontTarget,
                                         int leftBackTarget,
                                         int rightFrontTarget,
                                         int rightBackTarget)
    {
        int currentLeftFrontTarget = leftFront.getCurrentPosition() + leftFrontTarget;
        int currentLeftBackTarget = leftBack.getCurrentPosition() + leftBackTarget;
        int currentRightFrontTarget = rightFront.getCurrentPosition() + rightFrontTarget;
        int currentRightBackTarget = rightBack.getCurrentPosition() + rightBackTarget;

        leftFront.setTargetPosition(currentLeftFrontTarget);
        leftBack.setTargetPosition(currentLeftBackTarget);
        rightFront.setTargetPosition(currentRightFrontTarget);
        rightBack.setTargetPosition(currentRightBackTarget);

        // Turn On RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_TO_POSITION);

        Log.v("BOK", "START: LF: " + leftFront.getCurrentPosition() + ", " +
                currentLeftFrontTarget + ", LB: " +
                leftBack.getCurrentPosition() + ", " + currentLeftBackTarget + ", RF: " +
                rightFront.getCurrentPosition() + ",  " + currentRightFrontTarget + " RB: " +
                rightBack.getCurrentPosition() + ",  " + currentRightBackTarget);
    }

    /*
     * move() method: setup the robot to move encoder counts
     */
    public void startMove(double leftPower,
                          double rightPower,
                          double inches,
                          boolean forward)
    {
        double targetEncCount = getTargetEncCount(inches);
        if (forward) {
            setDTMotorEncoderTarget((int) targetEncCount, (int) -targetEncCount);
            setPowerToDTMotors(leftPower, leftPower, -rightPower, -rightPower);
        }
        else {
            setDTMotorEncoderTarget((int) -targetEncCount, (int) targetEncCount);
            setPowerToDTMotors(-leftPower, -leftPower, rightPower, rightPower);
        }
    }
    
    public void startStrafe(double power, double rotations, boolean right)
    {
        double targetEncCount = (rotations*COUNTS_PER_MOTOR_REV) * DRIVE_GEAR_REDUCTION;
        if (right) {
            setDTMotorEncoderTarget((int) targetEncCount, (int) -targetEncCount,
            (int) targetEncCount, (int) -targetEncCount);
            setPowerToDTMotors(power, -power, power, -power);
        }
        else {
            setDTMotorEncoderTarget((int) -targetEncCount, (int) targetEncCount,
            (int) -targetEncCount, (int) targetEncCount);
            setPowerToDTMotors(-power, power, -power, power);
        }
    }

    public void stopMove()
    {
        // Stop all motion;
        setPowerToDTMotors(0, 0, 0, 0);
        // Turn off RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean areDTMotorsBusy()
    {
        Log.v("BOK", "Current LF " + leftFront.getCurrentPosition() +
                ", RF " + rightFront.getCurrentPosition() +
                ", LB " + leftBack.getCurrentPosition() +
                ", RB " + rightBack.getCurrentPosition());
        return (leftFront.isBusy() &&
                rightFront.isBusy() &&
                leftBack.isBusy() &&
                rightBack.isBusy());
    }
}