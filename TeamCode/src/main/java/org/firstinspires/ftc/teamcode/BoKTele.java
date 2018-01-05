package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    private static final int TURNTABLE_COUNTS_PER_MOTOR_REV = 1120; // AndyMark 40
    private static final double UPPER_ARM_MOTOR_POWER_SLOW = 0.4;//0.2
    private static final double UPPER_ARM_MOTOR_POWER_FAST = 0.6;//0.4
    private static final double SPEED_COEFF_SLOW = 0.25;
    private static final double SPEED_COEFF_FAST = 0.5;
    private static final int RA_JOYSTICK_RATIO = 500;
    private static final int RELIC_DELAY_START = 40;
    private static final int RELIC_DEPLOY_STOP = 80; // loop runs 25 times a second
    private static final double RELIC_DEPLOY_POWER = 0.6;
    private static final float GLYPH_FLICKER_INCREMENT = 0.01F;

    private static int glyphArmInitialPos = 0;
    private static double glyphWristInitialPos = 0.0;


    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = SPEED_COEFF_FAST;
    private boolean trigger_left_decrease = false;

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
        robot.initializeImu();
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware()
    {
        boolean tank = false;
        boolean end_game = false;
        boolean relic_mode = false;
        boolean placementMode = false;

        double posOfUpperArm = 0;
        boolean clawClosed = false;

        boolean right_bumper_pressed = false;
        boolean left_bumper_pressed = false;
        boolean glyph_at_end = false;
        boolean relic_deploying = false;
        int relic_extend_delay_count = 0;

        robot.jewelArm.setPosition(robot.JA_INIT);
        robot.jewelFlicker.setPosition(robot.JF_INIT);
        robot.glyphArm.clawWrist.setPosition(robot.CW_INIT);
        robot.glyphArm.clawGrab.setPosition(robot.CG_OPEN);
        robot.turnTable.setPower(0);
        robot.turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // B:                  Go in tank mode
            // X:                  Go in mecanum mode
            // A:                  Go in slow mode
            // Y:                  Go in fast mode
            // Left bumper:        Raise glyph flicker
            // Right bumper:       Lower glyph flicker
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
            // Dpad Left:              Enter placement mode & raise the arm to 138 degrees
            // Dpad Right:             Exit placement mode
            // When in Placement Mode, use
            // Left bumper:            adjust upper arm position to 160 degrees
            // Right bumper:           adjust upper arm position to 180 degrees
            // X:                      adjust upper arm position to 138 degrees
            // When in End Game, use
            // Dpad Down:              Start Relic Mode
            // DPad Up:                End Relic Mode
            // Left Stick:             Relic Arm
            // Right Stick:            Relic Lift
            // B:                      Relic claw open
            // A:                      Relic claw closed
            // Left bumper:            Raise relic arm to clear the wall
            // Right bumper:           Lower relic arm for relic placement

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
                    // open the glyph flicker
                    robot.glyphFlicker.setPosition(robot.GF_FINAL);
                    // turn the relic arm
                    robot.relicArm.setPosition(robot.RA_DEPLOY_POS);
                    left_bumper_pressed = false;
                    right_bumper_pressed = false;
                    if (relic_extend_delay_count == 0) { // only once
                        // delay deploying the relic lift so that the wires are not tangled
                        relic_extend_delay_count = 1;
                    }
                    Log.v("BOK", "Relic mode started");
                }

                if (opMode.gamepad2.dpad_up) {
                    relic_mode = false;
                    relic_deploying = false;
                    Log.v("BOK", "Relic mode ended");
                }

                if (relic_extend_delay_count > 0) {
                    relic_extend_delay_count++;
                    relic_deploying = true; // automatic mode, use right stick for manual control

                    if ((relic_extend_delay_count >= RELIC_DELAY_START) &&
                        (relic_extend_delay_count < RELIC_DEPLOY_STOP)) {
                        if (relic_extend_delay_count == RELIC_DELAY_START) {
                            // deploy the relic lift
                            robot.relicSpool.setPower(RELIC_DEPLOY_POWER);
                        }
                    }
                    else if ((relic_extend_delay_count == RELIC_DEPLOY_STOP) ||
                             (relic_deploying == false)) {
                        robot.relicSpool.setPower(0.0);
                        relic_deploying = false; // end automatic mode
                        relic_extend_delay_count = -1; // stop the countdown
                    }
                }
            }

            if ((Math.abs(opMode.gamepad2.right_stick_y) >= TURNTABLE_STICK_DEAD_ZONE) ||
                    (Math.abs(opMode.gamepad2.right_stick_x) >= TURNTABLE_STICK_DEAD_ZONE)) {
                if (!relic_mode) {
                    double y = Math.abs(opMode.gamepad2.right_stick_y);
                    double x = opMode.gamepad2.right_stick_x;
                    double angle = Math.atan2(y, x);
                    angle = (angle * 180 / Math.PI);
                    opMode.telemetry.addData("Angle", angle);
                    opMode.telemetry.update();
                    //Log.v("BOK", "Angle: " + angle + " x: " + x + " y: " + y);
                    robot.turnTable.setTargetPosition(
                            (int)((TURNTABLE_COUNTS_PER_MOTOR_REV * (angle - 90))) / 180);
                    robot.turnTable.setPower(TURNTABLE_MOTOR_POWER);
                    //Log.v("BoK:", "Turntable Position: " + robot.turnTable.getCurrentPosition());
                }
            } else {
                if (!relic_mode && (robot.turnTable.getPower() != 0) && (!placementMode))
                    robot.turnTable.setPower(0.0);
            }

            if (!relic_mode) {
                if (!left_bumper_pressed && opMode.gamepad1.left_bumper) {
                    left_bumper_pressed = true;
                    robot.glyphFlicker.setPosition(robot.GF_INIT);
                }
                if (left_bumper_pressed){
                    if (robot.glyphFlicker.getPosition() < robot.GF_FINAL - 0.01) {
                        robot.glyphFlicker.setPosition(
                                robot.glyphFlicker.getPosition() + GLYPH_FLICKER_INCREMENT);
                    } else {
                        robot.glyphFlicker.setPosition(robot.GF_FINAL);
                        left_bumper_pressed = false; // stop moving the glyph flicker
                    }
                }

                if (opMode.gamepad1.right_bumper) { // close the glyph flicker
                    left_bumper_pressed = false; // ready to open again
                    robot.glyphFlicker.setPosition(robot.GF_INIT);
                }

                if (!placementMode) {
                    if (opMode.gamepad2.dpad_left || opMode.gamepad2.left_bumper ||
                            opMode.gamepad2.right_bumper) {
                        // enter placement mode & set to initial positions (if grabbed by tip of claw)
                        placementMode = true;
                        // move the turn table to straight position
                        glyphArmInitialPos = robot.upperArm.getCurrentPosition();
                        Log.v("BOK", "Dpad Left Arm Pos " + glyphArmInitialPos);
                        glyphWristInitialPos = robot.glyphClawWrist.getPosition();
                        robot.turnTable.setTargetPosition(0);
                        robot.turnTable.setPower(TURNTABLE_MOTOR_POWER);
                    }
                }

                if (placementMode) {
                    if (opMode.gamepad2.dpad_right) {
                        // exit placement mode
                        placementMode = false;
                        //robot.glyphFlicker.setPosition(robot.GF_INIT);
                        robot.glyphArm.clawWrist.setPosition(glyphWristInitialPos);
                        robot.glyphArm.moveUpperArmEncCount(glyphArmInitialPos,
                                0.8);
                    }
                    if (opMode.gamepad2.dpad_left) {
                        glyph_at_end = false;
                        //robot.glyphFlicker.setPosition(robot.GF_MID);
                        robot.glyphArm.moveUpperArmDegrees(robot.UA_GLYPH_AT_TIP, robot.UA_MOVE_POWER);
                        robot.glyphArm.clawWrist.setPosition(robot.CW_GLYPH_AT_TIP);
                    }
                    // position of upper arm if grabbed by middle of claw
                    if (opMode.gamepad2.left_bumper) {
                        glyph_at_end = false;
                        robot.glyphArm.moveUpperArmDegrees(robot.UA_GLYPH_AT_MID,
                                robot.UA_MOVE_POWER);
                        robot.glyphClawWrist.setPosition(robot.CW_GLYPH_AT_MID);
                    }
                    // position of upper arm if grabbed fully by claw
                    if (opMode.gamepad2.right_bumper) {
                        glyph_at_end = true;
                        robot.glyphArm.moveUpperArmDegrees(robot.UA_GLYPH_AT_END,
                                robot.UA_MOVE_POWER);
                        robot.glyphClawWrist.setPosition(robot.CW_GLYPH_AT_END);
                    }
                }
            }

            if (relic_mode) {
                if ((opMode.gamepad2.right_stick_y) <= -GAME_TRIGGER_DEAD_ZONE) {
                    relic_deploying = false; // manual control enabled!
                    robot.relicSpool.setPower(-opMode.gamepad2.right_stick_y*RELIC_DEPLOY_POWER);
                } else if (opMode.gamepad2.right_stick_y >= GAME_TRIGGER_DEAD_ZONE) {
                    relic_deploying = false; // manual control enabled!
                    robot.relicSpool.setPower(-opMode.gamepad2.right_stick_y*RELIC_DEPLOY_POWER);
                }
                else if (!relic_deploying) { // manual mode
                    robot.relicSpool.setPower(0);
                }
                if (!left_bumper_pressed && opMode.gamepad2.left_bumper ) {
                    //Log.v("BOK", "Left bumper pressed"); // raise the relic arm to RA_HIGH_POS
                    left_bumper_pressed = true;
                    robot.relicArm.setPosition(BoKHardwareBot.RA_HIGH_POS);
                }
                if (!right_bumper_pressed && opMode.gamepad2.right_bumper ) {
                    //Log.v("BOK", "Right bumper pressed"); // lower the relic arm to RA_DEPLOY_POS
                    right_bumper_pressed = true;
                    robot.relicArm.setPosition(BoKHardwareBot.RA_DEPLOY_POS-0.01);
                }
            }

            if (opMode.gamepad2.left_stick_y < -UPPER_ARM_STICK_DEAD_ZONE) {
                if (!end_game) {
                    stopUAAndResetGlyphFlicker(false);
                    placementMode = false;

                    double posOfArm = robot.upperArm.getCurrentPosition();
                    if (posOfArm < robot.glyphArm.ARM_AT_90_DEGREES_ENC_COUNT) {
                        double degreesChanged = (posOfArm - posOfUpperArm) *
                                robot.glyphArm.ARM_DEGREES_PER_ENC_COUNT;
                        robot.glyphArm.clawWrist.setPosition(
                                robot.glyphArm.clawWrist.getPosition() -
                                        (degreesChanged / robot.glyphArm.WRIST_SERVO_MAX_DEGREES));
                        posOfUpperArm = posOfArm;
                        //Log.v("BoK","Upper arm position: " + robot.upperArm.getCurrentPosition());
                    }
                    if (clawClosed)
                        robot.upperArm.setPower(UPPER_ARM_MOTOR_POWER_FAST);
                    else
                        robot.upperArm.setPower(UPPER_ARM_MOTOR_POWER_SLOW);
                    //Log.v("BOK", "Upper arm position: " + robot.upperArm.getCurrentPosition());
                } else if (relic_mode) {
                    //Log.v("BOK", "Relic mode Left Stick Up");
                    // Move relic arm up
                    double posOfArm = robot.relicArm.getPosition();
                    left_bumper_pressed = false;
                    right_bumper_pressed = false;
                    if (posOfArm < BoKHardwareBot.RA_UPPER_LIMIT) {
                    } else {
                        robot.relicArm.setPosition(posOfArm -
                                (-opMode.gamepad2.left_stick_y / RA_JOYSTICK_RATIO));
                    }
                }
            } else if (opMode.gamepad2.left_stick_y > UPPER_ARM_STICK_DEAD_ZONE) {
                if (!end_game) {
                    stopUAAndResetGlyphFlicker(false);
                    placementMode = false;

                    double posOfArm = robot.upperArm.getCurrentPosition();
                    double degreesChanged = (posOfArm - posOfUpperArm) * 0.064;
                    robot.glyphArm.clawWrist.setPosition(
                            robot.glyphArm.clawWrist.getPosition() - (degreesChanged / 240));
                    posOfUpperArm = posOfArm;

                    robot.upperArm.setPower(-UPPER_ARM_MOTOR_POWER_FAST);

                    //Log.v("BoK","Upper arm position: " + robot.upperArm.getCurrentPosition());
                } else if (relic_mode) {
                    // Move relic arm down
                    double posOfArm = robot.relicArm.getPosition();
                    left_bumper_pressed = false;
                    right_bumper_pressed = false;
                    if (posOfArm > BoKHardwareBot.RA_LOWER_LIMIT) {
                    } else {
                        robot.relicArm.setPosition(posOfArm +
                                (opMode.gamepad2.left_stick_y / RA_JOYSTICK_RATIO));
                    }
                }
            } else {
                if (robot.upperArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    stopUAAndResetGlyphFlicker(true);
                }
                else {
                    robot.upperArm.setPower(0);
                }
            }

            if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
                //Log.v("BoK","Wrist position: " + robot.glyphArm.clawWrist.getPosition());
                if (trigger_left_decrease)
                    robot.glyphArm.decreaseClawWristPos(opMode.gamepad2.left_trigger);
                else
                    robot.glyphArm.increaseClawWristPos(opMode.gamepad2.left_trigger);
            }

            if (opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) {
                //Log.v("BoK","Wrist position: " + robot.glyphArm.clawWrist.getPosition());
                if (trigger_left_decrease)
                    robot.glyphArm.increaseClawWristPos(opMode.gamepad2.right_trigger);
                else
                    robot.glyphArm.decreaseClawWristPos(opMode.gamepad2.right_trigger);
            }

            if (opMode.gamepad2.b) { // open the glyph claw grab or relic claw
                if (!relic_mode) {
                    robot.glyphArm.setClawGrabOpen();
                    if (glyph_at_end && placementMode) {
                        robot.glyphArm.moveUpperArmDegrees(robot.UA_GLYPH_AT_MID,
                                robot.UA_MOVE_POWER);
                        robot.glyphClawWrist.setPosition(robot.CW_GLYPH_AT_MID);
                        glyph_at_end = false;
                    }
                }
                else {
                    robot.relicClaw.setPosition(robot.RC_UNLOCK);
                }
                clawClosed = false;
            }

            if (opMode.gamepad2.a) { // close the glyph claw grab or relic claw
                if (!relic_mode) {
                    robot.glyphArm.setClawGrabClose();
                }
                else {
                    robot.relicClaw.setPosition(robot.RC_LOCK);
                }
                clawClosed = true;
            }

            robot.waitForTick(BoKHardwareBot.WAIT_PERIOD);
            //Log.v("BOK", "Distance of Glyph: " +
            //        robot.rangeSensorGA.getDistance(DistanceUnit.CM));
            //Log.v("BOK", "Distance of Glyph (opt): " +
            //        robot.rangeSensorGA.rawOptical());
        }

        return BoKTeleStatus.BOK_TELE_SUCCESS;
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

    private void stopUAAndResetGlyphFlicker(boolean checkIsBusy)
    {
        if (robot.upperArm.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            if (!checkIsBusy || (checkIsBusy && !robot.upperArm.isBusy())) {
                robot.upperArm.setPower(0);
                robot.upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.glyphFlicker.setPosition(robot.GF_INIT);
                Log.v("BOK", "Final Arm Pos " + robot.upperArm.getCurrentPosition());
            }
        }
    }
}
