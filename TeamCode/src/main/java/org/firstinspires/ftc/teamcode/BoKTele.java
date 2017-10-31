package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    private static final double GAME_STICK_DEAD_ZONE      = 0.05;
    private static final double GAME_TRIGGER_DEAD_ZONE    = 0.1;
    private static final double   COUNTS_PER_MOTOR_REV    = 1120;

    double posOfUpperArm = 0;
    int tTAngle = 0;
    double tTDegrees = 90;
    int tTAngleMax = 0;
    int tTAngleMin = -1120;
    int increment = 20;
    double currentPos;
    boolean clawClosed = false;

    private boolean tank = false;

    private double speedCoef = 0.5;

    public enum BoKTeleStatus
    {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }

    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot)
    {
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            if (opMode.gamepad1.x) {
                tank = false;
            }
            if (opMode.gamepad1.b) {
                tank = true;
            }
            if (!tank) {
                moveRobot(opMode, robot);
            } else {
                moveRobotTank(opMode, robot);
            }

            /*
             * Gamepad 1 controls
             */
            if (opMode.gamepad1.y) {
                speedCoef = 0.2;
            }
            if (opMode.gamepad1.a) {
                speedCoef = 0.5;
            }
            
            /*
             * Gamepad 2 controls
             */
            if ((Math.abs(opMode.gamepad2.right_stick_y) >= 0.8) ||
                    (Math.abs(opMode.gamepad2.right_stick_x) >= 0.8)) {
                double y = -opMode.gamepad2.right_stick_y;
                double x = opMode.gamepad2.right_stick_x;
                double angle = Math.atan2(y, x);
                angle = (-angle * 180 / Math.PI) + 90;
                if (angle >= 90 && angle <= 120) {
                    angle = 90;
                } else if (angle >= 240 && angle <= 270) {
                    angle = -90;
                }
                opMode.telemetry.addData("Angle", angle);
                robot.turnTable.setTargetPosition((int) ((1120 * (angle - 90))) / 180);
                opMode.telemetry.addData("Enc", (int) ((1120 * (angle - 90))) / 180);
                robot.turnTable.setPower(0.1);
            }
            else {

                robot.turnTable.setPower(0.0);
                robot.turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (opMode.gamepad2.left_stick_y < -0.2) {
                double posOfArm = robot.upperArm.getCurrentPosition();
                double degreesChanged = (posOfArm - posOfUpperArm) * 0.064;
                robot.glyphArm.clawWrist.setPosition(
                        robot.glyphArm.clawWrist.getPosition() - (degreesChanged / 240));
                posOfUpperArm = posOfArm;
                if (clawClosed)
                    robot.upperArm.setPower(0.4);
                else
                    robot.upperArm.setPower(0.2);
            } else if (opMode.gamepad2.left_stick_y > 0.2) {
                double posOfArm = robot.upperArm.getCurrentPosition();
                double degreesChanged = (posOfArm - posOfUpperArm) * 0.064;
                robot.glyphArm.clawWrist.setPosition(
                        robot.glyphArm.clawWrist.getPosition() - (degreesChanged / 240));
                if (clawClosed)
                    robot.upperArm.setPower(-0.4);
                else
                    robot.upperArm.setPower(-0.2);
                posOfUpperArm = posOfArm;
            } else {
                robot.upperArm.setPower(0);
                robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                double posOfArm = robot.upperArm.getCurrentPosition();
                //opMode.telemetry.addData("upperArm", posOfArm);
                //opMode.telemetry.update();
            }

            if (opMode.gamepad2.left_trigger > 0.2) {

                robot.glyphArm.increaseClawWristPos(-opMode.gamepad2.left_trigger);
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if (opMode.gamepad2.right_trigger > 0.2) {

                robot.glyphArm.decreaseClawWristPos(opMode.gamepad2.right_trigger);
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if (opMode.gamepad2.b) {

                robot.glyphArm.setClawGrabOpen();
                clawClosed = false;
                //Log.v("BOK","CLAWOPEN");

            }

            if (opMode.gamepad2.a) {

                robot.glyphArm.setClawGrabClose();
                clawClosed = true;
                //Log.v("BOK","CLAWCLOSED");

            }

            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (robot.flickerTouch.getState() == true) {
                //opMode.telemetry.addData("Digital Touch", "Is not pressed");
                //opMode.telemetry.update();
                //Log.v("BOK", "Hi");
            } else {
                Log.v("BOK", "Hi");
                //opMode.telemetry.addData("Digital Touch", "Is Pressed");
                //opMode.telemetry.update();
            }
/*
            if (opMode.gamepad1.a) {
                robot.spool.setPower(-0.15);
            } 
            if (opMode.gamepad1.b) {
                robot.spool.setPower(0.15);
            }
            if (opMode.gamepad1.y) {
                robot.spool.setPower(0);
            }

            if (opMode.gamepad1.dpad_up) {
                currentPos = robot.relicArm.getPosition() - 0.01;
                robot.relicArm.setPosition(currentPos);
            } else if (opMode.gamepad1.dpad_down) {
                currentPos = robot.relicArm.getPosition() + 0.01;
                robot.relicArm.setPosition(currentPos);
            }
*/

        }
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot(LinearOpMode opMode, BoKHardwareBot robot) {
        /*
         * Gamepad1: Driver 1 controls the robot using the left joystick for throttle and
         * the right joystick for steering
         */
        // NOTE: the left joystick goes negative when pushed upwards
        double gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        double gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        double gamePad1RightStickX = opMode.gamepad1.right_stick_x;

        double motorPowerLF = 0;
        double motorPowerLB = 0;
        double motorPowerRF = 0;
        double motorPowerRB = 0;

        //telemetry.addData("Throttle:",  "%.2f" + " Direction %.2f",
        // gamePad1LeftStickY, gamePad1RightStickX);
        //telemetry.update();
        //Log.v("BOK","moveRobot: " + String.format("%.2f", gamePad1LeftStickY) + ", " +
        //        String.format("%.2f", gamePad1LeftStickX) + ", " +
        //        String.format("%.2f", gamePad1RightStickX));

        // Run mecanum wheels

        if ((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) < -GAME_STICK_DEAD_ZONE) ) {
            motorPowerLF = -gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerLB = -gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerRF = gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerRB = gamePad1LeftStickY - gamePad1LeftStickX;

            Log.v("BOK","LF:" + String.format("%.2f", motorPowerLF) +
                    "LB: " + String.format("%.2f", motorPowerLB) +
                    "RF: " + String.format("%.2f", motorPowerRF) +
                    "RB: " + String.format("%.2f", motorPowerRB));
        }
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE)) {
            motorPowerLF = motorPowerLB = gamePad1RightStickX;
            motorPowerRF = motorPowerRB = gamePad1RightStickX;

            Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
                    "LB: " + String.format("%.2f", motorPowerLB) +
                    "RF: " + String.format("%.2f", motorPowerRF) +
                    "RB: " + String.format("%.2f", motorPowerRB));
        }
        // Right joystick is for turning


        /*if(motorPowerLB >=0){
            motorPowerLB = motorPowerLB*motorPowerLB;
        }
        else{
            motorPowerLB = -(motorPowerLB*motorPowerLB);
        }

        if(motorPowerLF >=0){
            motorPowerLF = motorPowerLF*motorPowerLF;
        }
        else{
            motorPowerLF = -(motorPowerLF*motorPowerLF);
        }

        if(motorPowerRB >=0){
            motorPowerRB = motorPowerRB*motorPowerRB;
        }
        else{
            motorPowerRB = -(motorPowerRB*motorPowerRB);
        }

        if(motorPowerRF >=0){
            motorPowerRF = motorPowerRF*motorPowerRF;
        }
        else{
            motorPowerRF = -(motorPowerLB*motorPowerRF);
        }*/
        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoef),
                (motorPowerLB * speedCoef),
                (motorPowerRF * speedCoef),
                (motorPowerRB * speedCoef));
    }
    public void moveRobotTank(LinearOpMode opMode, BoKHardwareBot robot) {
        double gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        double gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        double gamePad1RightStickX = opMode.gamepad1.right_stick_x;
        double gamePad1RightStickY = opMode.gamepad1.right_stick_y;

        double motorPowerLF = 0;
        double motorPowerLB = 0;
        double motorPowerRF = 0;
        double motorPowerRB = 0;

        if((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
        (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerLB = -gamePad1LeftStickY;
            motorPowerLF = -gamePad1LeftStickY;
        }

        if((Math.abs(gamePad1RightStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerRB = gamePad1RightStickY;
            motorPowerRF = gamePad1RightStickY;
        }
        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoef),
                (motorPowerLB * speedCoef),
                (motorPowerRF * speedCoef),
                (motorPowerRB * speedCoef));
    }
}
