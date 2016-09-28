package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public class BoK6WDHardwareBot extends BoKHardwareBot {
    // Constants
    private static final String LEFT_BACK_MOTOR_NAME = "lb";
    private static final String LEFT_FRONT_MOTOR_NAME = "lf";
    private static final String RIGHT_BACK_MOTOR_NAME = "rb";
    private static final String RIGHT_FRONT_MOTOR_NAME = "rf";

    // Drive train motors
    protected DcMotor leftBack;
    protected DcMotor leftFront;
    protected DcMotor rightBack;
    protected DcMotor rightFront;

    @Override
    /*
     * Initialize the drive system variables.
     * The init() method of the hardware class does all the work here
     */
    public BokStatus init(OpMode opMode) {

        leftBack = opMode.hardwareMap.dcMotor.get(LEFT_BACK_MOTOR_NAME);
        if (leftBack == null) {
            return BokStatus.BOK_FAILURE;
        }

        leftFront = opMode.hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR_NAME);
        if (leftFront == null) {
            return BokStatus.BOK_FAILURE;
        }

        rightBack = opMode.hardwareMap.dcMotor.get(RIGHT_BACK_MOTOR_NAME);
        if (rightBack == null) {
            return BokStatus.BOK_FAILURE;
        }

        rightFront = opMode.hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR_NAME);
        if (rightFront == null) {
            return BokStatus.BOK_FAILURE;
        }

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Drive train is initialized, initialize sensors
        return super.initSensors(opMode);
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
}
