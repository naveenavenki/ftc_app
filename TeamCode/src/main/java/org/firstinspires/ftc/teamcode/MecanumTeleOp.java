package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by shiv on 2/3/2017.
 */

public class MecanumTeleOp extends LeagueTeleopArcade {

    BoKMecanumWDHardwareBot robotWithMecanumWheels;

    protected double gamePad1LeftStickX;
    private double motorPowerFR;
    private double motorPowerFL;
    private double motorPowerBR;
    private double motorPowerBL;

    public void moveRobot(LinearOpMode opMode, BoKHardwareBot robot) {

        robotWithMecanumWheels = (BoKMecanumWDHardwareBot)robot;

        /*
         * Gamepad1: Driver 1 controls the robot using the left joystick for throttle and
         * the right joystick for steering
         */
        // NOTE: the left joystick goes negative when pushed upwards
        gamePad1LeftStickY = -opMode.gamepad1.left_stick_y;
        gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        gamePad1RightStickX = opMode.gamepad1.right_stick_x;


        //telemetry.addData("Throttle:",  "%.2f" + " Direction %.2f",
        // gamePad1LeftStickY, gamePad1RightStickX);
        //telemetry.addData("Power:", "left: %.2f" + " right: %.2f", leftDTPower, rightDTPower);
        //telemetry.update();

        // Run mecanum wheels

        // Left joystick is for throttle
        if (((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) &&
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE)) ||
                (Math.abs(gamePad1LeftStickX) > GAME_STICK_DEAD_ZONE) &&
                (Math.abs(gamePad1LeftStickX) < -GAME_STICK_DEAD_ZONE) )
        {
            motorPowerFR = gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerFL = -gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerBR = -gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerBL = gamePad1LeftStickY - gamePad1LeftStickX;

            Log.v("BOK","FL:" + motorPowerFL + "FR: " + motorPowerFR + "BL: " + motorPowerBL +
                    "BR: " + motorPowerBR);

            robotWithMecanumWheels.setPowerToMecanumDTMotors(
                    motorPowerFL,
                    motorPowerBL,
                    motorPowerFR,
                    motorPowerBR);


        }

        // Right joystick is for steering
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) &&
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerFR = -gamePad1RightStickX;
            motorPowerFL = -gamePad1RightStickX;
            motorPowerBR = gamePad1RightStickX;
            motorPowerBL = gamePad1RightStickX;

            Log.v("BOK","FL:" + motorPowerFL + "FR: " + motorPowerFR + "BL: " + motorPowerBL +
                    "BR: " + motorPowerBR);

            robotWithMecanumWheels.setPowerToMecanumDTMotors(
                    motorPowerFL,
                    motorPowerBL,
                    motorPowerFR,
                    motorPowerBR);
        }
        else {
            robotWithMecanumWheels.setPowerToMecanumDTMotors(0, 0, 0, 0);

        }
    }

}

