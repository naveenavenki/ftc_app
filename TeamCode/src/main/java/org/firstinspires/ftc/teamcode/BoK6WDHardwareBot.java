package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public class BoK6WDHardwareBot extends BoKHardwareBot {
    // 280 cycles per revolution (CPR); It is a quadrature encoder producing 4 Pulses per Cycle. With 280 CPR, it outputs 1120 PPR.
    protected static final double   COUNTS_PER_MOTOR_REV    = 1120;   // AndyMark 40 Motor Encoder
    protected static final double   DRIVE_GEAR_REDUCTION    = 1.33;   // For 360 degrees wheel turn, motor shaft moves 480 degrees (approx)
    protected static final double   WHEEL_DIAMETER_INCHES   = 4.0;    // For calculating circumference
    protected static final double   WHEEL_BASE              = 15;     // In inches (approx)
    protected static final double   COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final int DISTANCE_THRESHOLD             = 10;

    // Constants
    private static final String LEFT_BACK_MOTOR_NAME   = "lb";
    private static final String LEFT_FRONT_MOTOR_NAME  = "lf";
    private static final String RIGHT_BACK_MOTOR_NAME  = "rb";
    private static final String RIGHT_FRONT_MOTOR_NAME = "rf";


    // Drive train motors
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    // Encoder
    private int currentLeftTarget;
    private int currentRightTarget;
    private boolean leftPositive;
    private boolean rightPositive;
    private boolean leftReached;
    private boolean rightReached;

    @Override
    /*
     * Initialize the drive system variables.
     * The init() method of the hardware class does all the work here
     */
    protected BoKStatus initMotors(LinearOpMode opMode) {
        currentOpMode = opMode;

        leftBack = opMode.hardwareMap.dcMotor.get(LEFT_BACK_MOTOR_NAME);
        if (leftBack == null) {
            return BoKStatus.BOK_FAILURE;
        }

        leftFront = opMode.hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR_NAME);
        if (leftFront == null) {
            return BoKStatus.BOK_FAILURE;
        }

        rightBack = opMode.hardwareMap.dcMotor.get(RIGHT_BACK_MOTOR_NAME);
        if (rightBack == null) {
            return BoKStatus.BOK_FAILURE;
        }

        rightFront = opMode.hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR_NAME);
        if (rightFront == null) {
            return BoKStatus.BOK_FAILURE;
        }

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Drive train is initialized, initialize sensors
        return BoKStatus.BOK_SUCCESS;
    }

    public void setPowerToMotors(double left, double right) {
        leftBack.setPower(left);
        rightBack.setPower(right);
        leftFront.setPower(left);
        rightFront.setPower(right);
        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS);
    }

    public void setModeForMotors(DcMotor.RunMode runMode)
    {
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS);
    }

    public void setMotorEncoderTarget(int leftTarget, int rightTarget)
    {
        setModeForMotors(DcMotor.RunMode.RUN_USING_ENCODER);

        leftReached = rightReached = false;

        currentLeftTarget = leftFront.getCurrentPosition() + leftTarget;
        currentRightTarget = rightFront.getCurrentPosition() + rightTarget;
        //leftBack.setTargetPosition(leftBack.getCurrentPosition() + leftTarget);
        //rightBack.setTargetPosition(rightBack.getCurrentPosition() + rightTarget);
        leftFront.setTargetPosition(currentLeftTarget);
        rightFront.setTargetPosition(currentRightTarget);
        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS);
        //opMode.idle();

        if (leftTarget > 0)
            leftPositive = true;
        else
            leftPositive = false;

        if (rightTarget > 0)
            rightPositive = true;
        else
            rightPositive = false;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS);
        //leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Log.v("BOK", "START: " + leftFront.getCurrentPosition() +", " + currentLeftTarget + ", " + rightFront.getCurrentPosition() + ", " + currentRightTarget);
    }

    // Returns true if target is reached
    public boolean getCurrentPosition()
    {
        //boolean leftReached = false;
        //boolean rightReached = false;
        int leftFrontCurrentPos = leftFront.getCurrentPosition();
        int rightFrontCurrentPos = rightFront.getCurrentPosition();

        //Log.v("BOK", "Current " + leftFrontCurrentPos + ", " + leftFront.isBusy() + ", " + rightFrontCurrentPos + ", " + rightFront.isBusy());

        if (leftPositive) {
            if ((leftFrontCurrentPos >= currentLeftTarget) || (Math.abs(leftFrontCurrentPos - currentLeftTarget) <= DISTANCE_THRESHOLD)) {
                Log.v("BOK", "RETURNING TRUE!!");
                leftReached = true;
                //return true;
            }
        }
        else {
            if ((leftFrontCurrentPos <= currentLeftTarget) || (Math.abs(leftFrontCurrentPos - currentLeftTarget) <= DISTANCE_THRESHOLD))
                leftReached = true;
                //return true;
        }

        if (rightPositive) {
            if ((rightFrontCurrentPos >= currentRightTarget) || (Math.abs(rightFrontCurrentPos - currentRightTarget) <= DISTANCE_THRESHOLD)) {
                Log.v("BOK", "RETURNING TRUE right!!");
                //return true;
                rightReached = true;
            }
        }
        else {
            if ((rightFrontCurrentPos <= currentRightTarget) || (Math.abs(rightFrontCurrentPos - currentRightTarget) <= DISTANCE_THRESHOLD))
                rightReached = true;
                //return true;
        }

        return rightReached && leftReached;
    }
}
