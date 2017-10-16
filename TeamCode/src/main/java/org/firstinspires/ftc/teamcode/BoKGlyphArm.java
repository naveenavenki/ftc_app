package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by shiv on 10/6/2017.
 */

public class BoKGlyphArm {

    BoKHardwareBot robot;
    public BoKGlyphArm(BoKHardwareBot robot)
    {
        this.robot = robot;
    }
    private static final double   COUNTS_PER_MOTOR_REV    = 1120;
    private static final double   DRIVE_GEAR_REDUCTION    = 5.0;

    private double getTargetEncCount(double targetAngleDegrees)
    {
        double degreesOfMotorTurn = DRIVE_GEAR_REDUCTION * targetAngleDegrees;
        return (COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;
    }

    public int moveUpperArm(double targetAngleDegrees, double power) {

        robot.upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) getTargetEncCount(targetAngleDegrees);
        Log.v("BOK", "Target: " + target);
/*
        double posD = robot.clawWrist.getPosition();
        if (target > 0) {
            robot.clawWrist.setPosition(posD - targetAngleDegrees/180.0);
        }
        else {
            robot.clawWrist.setPosition(posD + targetAngleDegrees/180.0);
        }
*/
        robot.upperArm.setTargetPosition((int)getTargetEncCount(targetAngleDegrees));
        robot.upperArm.setPower(power);
        return target;
    }

    public void increaseClawWristPos()
    {
        double pos = robot.clawWrist.getPosition();

        if(pos > robot.CW_INIT) {

        }
        else {
            robot.clawWrist.setPosition(pos + 0.001);
        }
    }

    public void decreaseClawWristPos()
    {
        double pos = robot.clawWrist.getPosition();

        if(pos < 0.1) {
        }
        else {
            robot.clawWrist.setPosition(pos - 0.001);
        }
    }

    public void setClawGrabOpen(){

        robot.clawGrab.setPosition(0);
    }

    public void setClawGrabClose(){

        robot.clawGrab.setPosition(1);
    }

}
