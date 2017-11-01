package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Krishna Saxena on 10/6/2017.
 */

public class BoKGlyphArm {

    BoKHardwareBot robot;
    LinearOpMode opMode;
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
    
    private static final double   COUNTS_PER_MOTOR_REV    = 1120;
    private static final double   DRIVE_GEAR_REDUCTION    = 5.0;

    private double getTargetEncCount(double targetAngleDegrees)
    {
        double degreesOfMotorTurn = DRIVE_GEAR_REDUCTION * targetAngleDegrees;
        return (COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;
    }

    public void moveUpperArm(double targetAngleDegrees, double power) {

        robot.upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) getTargetEncCount(targetAngleDegrees);
        Log.v("BOK", "Target: " + target);

        robot.upperArm.setTargetPosition((int)getTargetEncCount(targetAngleDegrees));
        robot.upperArm.setPower(power);
        while (opMode.opModeIsActive() &&
                /*(robot.getDTCurrentPosition() == false) &&*/
                robot.upperArm.isBusy()) {
            //opMode.telemetry.update();
            opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }
        robot.upperArm.setPower(0);
        robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void increaseClawWristPos(double stick_y)
    {
        double pos = clawWrist.getPosition();

        if(pos > robot.CW_INIT) {

        }
        else {
            clawWrist.setPosition(pos - stick_y/100);
        }
    }

    public void decreaseClawWristPos(double stick_y)
    {
        double pos = clawWrist.getPosition();

        if(pos < 0.1) {
        }
        else {
            clawWrist.setPosition(pos - stick_y/100);
        }
    }

    public void setClawGrabOpen(){

        clawGrab.setPosition(BoKHardwareBot.CG_MID);
    }

    public void setClawGrabClose(){

        clawGrab.setPosition(BoKHardwareBot.CG_CLOSE);
    }

}
