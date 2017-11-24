package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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
    private static final double SPEED_COEFF_SLOW = 0.25;
    private static final double SPEED_COEFF_FAST = 0.5;
    private static final int JOYSTICK_RATIO = 100;

    private BoKHardwareBot robot;
    private LinearOpMode opMode;

    private double posOfUpperArm = 0;
    private boolean clawClosed = false;

    private boolean tank = false;
    private boolean end_game = false;
    private boolean relic_mode = false;
    private boolean trigger_left_decrease = false;
    private double speedCoef = SPEED_COEFF_FAST;

    public enum BoKTeleStatus
    {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }

    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot,
                                      boolean trigger_left_decrease)
    {
        this.trigger_left_decrease = trigger_left_decrease;
        this.opMode = opMode;
        this.robot = robot;
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware()
    {
        robot.turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                moveRobot();
            } else {
                moveRobotTank();
            }

            if (opMode.gamepad1.y) {
                speedCoef = SPEED_COEFF_SLOW;
            }
            if (opMode.gamepad1.a) {
                speedCoef = SPEED_COEFF_FAST;
            }

            // GAMEPAD 2 CONTROLS
            // Left stick:             Upper Arm
            // Right stick:            Turntable
            // Left and right trigger: Claw wrist down and up
            // B:                      Claw open
            // A:                      Claw closed
            // Y:                      End Game
            // X:                      Not End Game
            // When in End Game, use
            // Dpad Down:              Start Relic Game
            // DPad Up:                End Relic Mode
            // Left Stick:             Relic Arm
            // Right Stick:            Relic Lift
            if (opMode.gamepad2.y) {
                end_game = true;
                Log.v("BOK", "End Game Started");
            }
            if (opMode.gamepad2.x) {
                end_game = false;
            }
            if (end_game) { // You need to be in end_game to be in relic mode
                if (opMode.gamepad2.dpad_down) {
                    relic_mode = true;
                    Log.v("BOK", "Relic Mode Started");
                }
                if (opMode.gamepad2.dpad_up) {
                    relic_mode = false;
                }
            }

            if ((Math.abs(opMode.gamepad2.right_stick_y) >= TURNTABLE_STICK_DEAD_ZONE) ||
                    (Math.abs(opMode.gamepad2.right_stick_x) >= TURNTABLE_STICK_DEAD_ZONE)) {
                if (!relic_mode) {
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
            } else {
                if (!relic_mode && (robot.turnTable.getPower() != 0))
                    robot.turnTable.setPower(0.0);
                //robot.turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (relic_mode) {
                if ((opMode.gamepad2.right_stick_y) >= GAME_TRIGGER_DEAD_ZONE) {
                    if (robot.spool.getPosition() >= 0.06)
                        robot.spool.setPosition(robot.spool.getPosition()-0.01);
                } else if (opMode.gamepad2.right_stick_y <= -GAME_TRIGGER_DEAD_ZONE) {
                    if (robot.spool.getPosition() < robot.SP_INIT)
                        robot.spool.setPosition(robot.spool.getPosition()+0.01);
                }
            }

            if (opMode.gamepad2.left_stick_y < -UPPER_ARM_STICK_DEAD_ZONE) {
                if (!end_game) {
                    double posOfArm = robot.upperArm.getCurrentPosition();
                    double degreesChanged = (posOfArm - posOfUpperArm) * 0.064;
                    robot.glyphArm.clawWrist.setPosition(
                            robot.glyphArm.clawWrist.getPosition() - (degreesChanged / 240));
                    posOfUpperArm = posOfArm;
                    if (clawClosed)
                        robot.upperArm.setPower(UPPER_ARM_MOTOR_POWER_FAST);
                    else
                        robot.upperArm.setPower(UPPER_ARM_MOTOR_POWER_SLOW);
                } else if (relic_mode) {
                    Log.v("BOK", "Relic mode Left Stick Up");
                    // Move relic arm up
                    double posOfArm = robot.relicArm.getPosition();
                    if (posOfArm > 1) {
                    } else {
                        robot.relicArm.setPosition(posOfArm +
                                (-opMode.gamepad2.left_stick_y / JOYSTICK_RATIO));
                    }
                }
            } else if (opMode.gamepad2.left_stick_y > UPPER_ARM_STICK_DEAD_ZONE) {
                if (!end_game) {
                    double posOfArm = robot.upperArm.getCurrentPosition();
                    double degreesChanged = (posOfArm - posOfUpperArm) * 0.064;
                    robot.glyphArm.clawWrist.setPosition(
                            robot.glyphArm.clawWrist.getPosition() - (degreesChanged / 240));
                    posOfUpperArm = posOfArm;
                    if (clawClosed)
                        robot.upperArm.setPower(-UPPER_ARM_MOTOR_POWER_FAST);
                    else
                        robot.upperArm.setPower(-UPPER_ARM_MOTOR_POWER_SLOW);
                } else if (relic_mode) {
                    Log.v("BOK", "Relic mode Left Stick Down");
                    // Move relic arm down
                    double posOfArm = robot.relicArm.getPosition();
                    if (posOfArm < BoKHardwareBot.RA_INIT) {
                    } else {
                        robot.relicArm.setPosition(posOfArm -
                                (opMode.gamepad2.left_stick_y / JOYSTICK_RATIO));
                    }
                }
            } else {
                robot.upperArm.setPower(0);
                //robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //double posOfArm = robot.upperArm.getCurrentPosition();
                //opMode.telemetry.addData("upperArm", posOfArm);
                //opMode.telemetry.update();
            }

            if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
                if (trigger_left_decrease)
                    robot.glyphArm.decreaseClawWristPos(opMode.gamepad2.left_trigger);
                else
                    robot.glyphArm.increaseClawWristPos(opMode.gamepad2.left_trigger);
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if (opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) {
                if (trigger_left_decrease)
                    robot.glyphArm.increaseClawWristPos(opMode.gamepad2.right_trigger);
                else
                    robot.glyphArm.decreaseClawWristPos(opMode.gamepad2.right_trigger);
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

            robot.waitForTick(BoKHardwareBot.WAIT_PERIOD);
        }
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRelicArm(boolean down)
    {
        if (down) {
            for (double i = robot.relicArm.getPosition(); i >= 0.0; i -= 0.01) {
                robot.relicArm.setPosition(i);
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            }
        }
        else {
            for (double i = robot.relicArm.getPosition(); i <= 1; i += 0.01) {
                robot.relicArm.setPosition(i);
                opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
            }
        }
    }

    private void moveRobot()
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

            //Log.v("BOK","LF:" + String.format("%.2f", motorPowerLF*speedCoef) +
            //        "LB: " + String.format("%.2f", motorPowerLB*speedCoef) +
            //        "RF: " + String.format("%.2f", motorPowerRF*speedCoef) +
            //        "RB: " + String.format("%.2f", motorPowerRB*speedCoef));
        }
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE)) {
            // Right joystick is for turning
            motorPowerLF = motorPowerLB = gamePad1RightStickX;
            motorPowerRF = motorPowerRB = gamePad1RightStickX;

            //Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
            //        "LB: " + String.format("%.2f", motorPowerLB) +
            //        "RF: " + String.format("%.2f", motorPowerRF) +
            //        "RB: " + String.format("%.2f", motorPowerRB));
        }
        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoef),
                (motorPowerLB * speedCoef),
                (motorPowerRF * speedCoef),
                (motorPowerRB * speedCoef));
    }

    private void moveRobotTank()
    {
        double gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
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
