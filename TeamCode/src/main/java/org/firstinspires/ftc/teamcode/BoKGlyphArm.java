package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Krishna Saxena on 10/6/2017.
 */

public class BoKGlyphArm
{
    // CONSTANTS
    private static final double COUNTS_PER_MOTOR_REV    = 1120; // AndyMark 40
    private static final double DRIVE_GEAR_REDUCTION    = 5.0;
    protected static final double ARM_DEGREES_PER_ENC_COUNT = 0.064;
    protected static final int ARM_AT_90_DEGREES_ENC_COUNT = 1400;
    protected static final int WRIST_SERVO_MAX_DEGREES = 240; // programmed for Rev Smart Servo
    private static final int WRIST_JOYSTICK_RATIO = 100;

    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    protected Servo clawWrist;
    protected Servo clawGrab;
    
    public BoKGlyphArm(BoKHardwareBot robot,
                       LinearOpMode opMode,
                       Servo clawWrist,
                       Servo clawGrab)
    {
        this.robot = robot;
        this.opMode = opMode;
        this.clawWrist = clawWrist;
        this.clawGrab = clawGrab;
    }

     private double getTargetEncCount(double targetAngleDegrees)
    {
        double degreesOfMotorTurn = DRIVE_GEAR_REDUCTION * targetAngleDegrees;
        return (COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;
    }

    public void moveUpperArm(double targetAngleDegrees, double power)
    {
        robot.upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) getTargetEncCount(targetAngleDegrees);
        Log.v("BOK", "Target (arm): " + target);

        robot.upperArm.setTargetPosition((int)getTargetEncCount(targetAngleDegrees));
        robot.upperArm.setPower(power);
        while (opMode.opModeIsActive() && robot.upperArm.isBusy()) {
            //opMode.telemetry.update();
            opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }
        robot.upperArm.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveUpperArmDegrees(double targetAngleDegrees, double power)
    {
        int target = (int) getTargetEncCount(targetAngleDegrees);
        Log.v("BOK", "TargetD (arm enc): " + target + ", current: " +
                robot.upperArm.getCurrentPosition());

        robot.upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperArm.setTargetPosition(target);
        robot.upperArm.setPower(power);
    }

    public void moveUpperArmEncCount(int targetEncCount, double power)
    {
        Log.v("BOK", "TargetE (arm enc): " + targetEncCount + ", " +
                "current: " + robot.upperArm.getCurrentPosition());

        robot.upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperArm.setTargetPosition(targetEncCount);
        robot.upperArm.setPower(power);
    }

    public void increaseClawWristPos(double trigger)
    {
        double pos = clawWrist.getPosition();

        if(pos > 1) {
        }
        else {
            clawWrist.setPosition(pos + trigger/WRIST_JOYSTICK_RATIO);
        }
        //Log.v("BOK", "Wrist Pos" + clawWrist.getPosition());
    }

    public void decreaseClawWristPos(double trigger)
    {
        double pos = clawWrist.getPosition();
        double min = robot.CW_MIN;
        if (robot.upperArm.getCurrentPosition() < 450) // approx 30 degree
            min = 0.25;

        if(pos < min) {
        }
        else {
            clawWrist.setPosition(pos - trigger/WRIST_JOYSTICK_RATIO);
        }
        //Log.v("BOK", "Wrist Pos" + clawWrist.getPosition());
    }

    public void setClawGrabOpen()
    {
        clawGrab.setPosition(BoKHardwareBot.CG_OPEN);
    }

    public void setClawGrabClose()
    {
        clawGrab.setPosition(BoKHardwareBot.CG_CLOSE);
    }
}
