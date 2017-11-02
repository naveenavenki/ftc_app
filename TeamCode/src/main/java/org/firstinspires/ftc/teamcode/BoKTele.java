package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    // CONSTANTS
    private static final double GAME_STICK_DEAD_ZONE = 0.05;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double TURNTABLE_STICK_DEAD_ZONE = 0.8;
    private static final double UPPER_ARM_STICK_DEAD_ZONE = 0.2;
    private static final double TURNTABLE_MOTOR_POWER = 0.2;
    private static final double UPPER_ARM_MOTOR_POWER_SLOW = 0.2;
    private static final double UPPER_ARM_MOTOR_POWER_FAST = 0.4;
    private static final double SPEED_COEFF_SLOW = 0.5;
    private static final double SPEED_COEFF_FAST = 0.2;
    private static final int WAIT_PERIOD = 40;

    private double posOfUpperArm = 0;
    private boolean clawClosed = false;

    private boolean tank = false;
    private double speedCoef = SPEED_COEFF_FAST;

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
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // B:                  Go in tank mode
            // X:                  Go out of tank mode
            // A:                  Go in slow mode
            // Y:                  Go in fast mode
            if (opMode.gamepad1.b) {
                tank = true;
            }
            if (opMode.gamepad1.x) {
                tank = false;
            }
            if (!tank) {
                moveRobot(opMode, robot);
            } else {
                moveRobotTank(opMode, robot);
            }

            if (opMode.gamepad1.y) {
                speedCoef = SPEED_COEFF_FAST;
            }
            if (opMode.gamepad1.a) {
                speedCoef = 0.5;
            }
            
            // GAMEPAD 2 CONTROLS
            // Left stick:             Upper Arm
            // Right stick:            Turntable
            // Left and right trigger: Claw wrist down and up
            // B:                      Claw open
            // A:                      Claw closed
            if ((Math.abs(opMode.gamepad2.right_stick_y) >= TURNTABLE_STICK_DEAD_ZONE) ||
                    (Math.abs(opMode.gamepad2.right_stick_x) >= TURNTABLE_STICK_DEAD_ZONE)) {
                double y = -opMode.gamepad2.right_stick_y;
                double x = opMode.gamepad2.right_stick_x;
                double angle = Math.atan2(y, x);
                angle = (-angle * 180 / Math.PI) + 90;
                if (angle >= 90 && angle <= 120) {
                    angle = 90;
                } else if (angle >= 240 && angle <= 270) {
                    angle = -90;
                }
                //opMode.telemetry.addData("Angle", angle);
                robot.turnTable.setTargetPosition((int) ((1120 * (angle - 90))) / 180);
                //opMode.telemetry.addData("Enc", (int) ((1120 * (angle - 90))) / 180);
                robot.turnTable.setPower(TURNTABLE_MOTOR_POWER);
            }
            else {

                robot.turnTable.setPower(0.0);
                robot.turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (opMode.gamepad2.left_stick_y < -UPPER_ARM_STICK_DEAD_ZONE) {
                double posOfArm = robot.upperArm.getCurrentPosition();
                double degreesChanged = (posOfArm - posOfUpperArm) * 0.064;
                robot.glyphArm.clawWrist.setPosition(
                        robot.glyphArm.clawWrist.getPosition() - (degreesChanged / 240));
                posOfUpperArm = posOfArm;
                if (clawClosed)
                    robot.upperArm.setPower(UPPER_ARM_MOTOR_POWER_FAST);
                else
                    robot.upperArm.setPower(UPPER_ARM_MOTOR_POWER_SLOW);
            } else if (opMode.gamepad2.left_stick_y > UPPER_ARM_STICK_DEAD_ZONE) {
                double posOfArm = robot.upperArm.getCurrentPosition();
                double degreesChanged = (posOfArm - posOfUpperArm) * 0.064;
                robot.glyphArm.clawWrist.setPosition(
                        robot.glyphArm.clawWrist.getPosition() - (degreesChanged / 240));
                if (clawClosed)
                    robot.upperArm.setPower(-UPPER_ARM_MOTOR_POWER_FAST);
                else
                    robot.upperArm.setPower(-UPPER_ARM_MOTOR_POWER_SLOW);
                posOfUpperArm = posOfArm;
            } else {
                robot.upperArm.setPower(0);
                robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                double posOfArm = robot.upperArm.getCurrentPosition();
                //opMode.telemetry.addData("upperArm", posOfArm);
                //opMode.telemetry.update();
            }

            if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
               robot.glyphArm.decreaseClawWristPos(-opMode.gamepad2.left_trigger);
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if (opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) {
                robot.glyphArm.increaseClawWristPos(opMode.gamepad2.right_trigger);
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if (opMode.gamepad2.b) {
                robot.glyphArm.setClawGrabOpen();
                clawClosed = false;
            }

            if (opMode.gamepad2.a) {
                robot.glyphArm.setClawGrabClose();
                clawClosed = true;
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
            robot.waitForTick(WAIT_PERIOD);
        }
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot(LinearOpMode opMode, BoKHardwareBot robot)
    {
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
        boolean moving = false;

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
            moving = true;

            Log.v("BOK","LF:" + String.format("%.2f", motorPowerLF) +
                    "LB: " + String.format("%.2f", motorPowerLB) +
                    "RF: " + String.format("%.2f", motorPowerRF) +
                    "RB: " + String.format("%.2f", motorPowerRB));
        }
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE)) {
            // Right joystick is for turning
            motorPowerLF = motorPowerLB = gamePad1RightStickX;
            motorPowerRF = motorPowerRB = gamePad1RightStickX;
            moving = true;

            Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
                    "LB: " + String.format("%.2f", motorPowerLB) +
                    "RF: " + String.format("%.2f", motorPowerRF) +
                    "RB: " + String.format("%.2f", motorPowerRB));
        }
        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoef),
                (motorPowerLB * speedCoef),
                (motorPowerRF * speedCoef),
                (motorPowerRB * speedCoef));
        if (!moving) {
            robot.setZeroPowerBehaviorDTMotors();
        }
    }

    public void moveRobotTank(LinearOpMode opMode, BoKHardwareBot robot)
    {
        double gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        double gamePad1RightStickY = opMode.gamepad1.right_stick_y;

        double motorPowerLF = 0;
        double motorPowerLB = 0;
        double motorPowerRF = 0;
        double motorPowerRB = 0;
        boolean moving = false;

        if((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
        (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerLB = -gamePad1LeftStickY;
            motorPowerLF = -gamePad1LeftStickY;
            moving = true;
        }

        if((Math.abs(gamePad1RightStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerRB = gamePad1RightStickY;
            motorPowerRF = gamePad1RightStickY;
            moving = true;
        }
        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoef),
                (motorPowerLB * speedCoef),
                (motorPowerRF * speedCoef),
                (motorPowerRB * speedCoef));
        if (!moving) {
            robot.setZeroPowerBehaviorDTMotors();
        }
    }
}
