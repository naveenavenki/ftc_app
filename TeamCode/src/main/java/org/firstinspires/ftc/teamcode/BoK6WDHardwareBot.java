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

    @Override
    /*
     * Initialize the drive system variables.
     * The init() method of the hardware class does all the work here
     */
    protected BoKStatus initMotors(OpMode opMode) {

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
        leftFront.setPower(left);
        rightBack.setPower(right);
        rightFront.setPower(right);
    }

    public void setModeForMotors(DcMotor.RunMode runMode)
    {
        leftBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightBack.setMode(runMode);
        rightFront.setMode(runMode);
    }

    public void setupMotorEncoders(LinearOpMode opMode)
    {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.idle();

        // Send telemetry message to indicate successful Encoder reset
        opMode.telemetry.addData("BOK",  "Starting at %5d %5d %5d %5d",
                leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
        opMode.telemetry.update();
        Log.v("BOK", "Start: " + leftFront.getCurrentPosition() +", " + rightFront.getCurrentPosition() + ", " + leftBack.getCurrentPosition() + ", " + rightBack.getCurrentPosition());
    }

    public void setMotorEncoderTarget(int leftTarget, int rightTarget)
    {
        currentLeftTarget = leftFront.getCurrentPosition() + leftTarget;
        leftFront.setTargetPosition(currentLeftTarget);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + leftTarget);
        if (leftTarget > 0)
            leftPositive = true;
        else
            leftPositive = false;

        currentRightTarget = rightFront.getCurrentPosition() + rightTarget;
        rightFront.setTargetPosition(currentRightTarget);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rightTarget);
        if (rightTarget > 0)
            rightPositive = true;
        else
            rightPositive = false;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Log.v("BOK", "START: " + leftFront.getCurrentPosition() +", " + currentLeftTarget + ", " + rightFront.getCurrentPosition() + ", " + currentRightTarget);
    }

    // Returns true if target is reached
    public boolean getCurrentPosition(OpMode opMode)
    {
        int leftFrontCurrentPos = leftFront.getCurrentPosition();
        int rightFrontCurrentPos = rightFront.getCurrentPosition();
        //int leftBackCurrentPos = leftBack.getCurrentPosition();
        //int rightBackCurrentPos = rightBack.getCurrentPosition();


        opMode.telemetry.addData("Target",  "Running to %5d : %5d", currentLeftTarget,  currentRightTarget);
        opMode.telemetry.addData("Position",  "Running at %5d %5d", leftFrontCurrentPos, rightFrontCurrentPos);
        opMode.telemetry.update();

        //Log.v("BOK", "Target " + currentLeftTarget + ", " + currentRightTarget);
        //Log.v("BOK", "Current " + leftFrontCurrentPos + ", " + rightFrontCurrentPos);

        if (leftPositive) {
            if (leftFrontCurrentPos >= currentLeftTarget)
                return true;
        }
        else {
            if (leftFrontCurrentPos <= currentLeftTarget)
                return true;
        }
        if (rightPositive) {
            if (rightFrontCurrentPos >= currentRightTarget)
                return true;
        }
        else {
            if (rightFrontCurrentPos <= currentRightTarget)
                return true;
        }
        return false;
    }
}
