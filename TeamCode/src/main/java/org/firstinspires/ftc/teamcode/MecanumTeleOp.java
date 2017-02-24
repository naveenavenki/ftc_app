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

    BoK4MotorsDTBot robotWithMecanumWheels;

    protected double gamePad1LeftStickX;
    private double motorPowerLF;
    private double motorPowerLB;
    private double motorPowerRF;
    private double motorPowerRB;

    public void moveRobot(LinearOpMode opMode, BoKHardwareBot robot) {

        robotWithMecanumWheels = (BoK4MotorsDTBot)robot;

        /*
         * Gamepad1: Driver 1 controls the robot using the left joystick for throttle and
         * the right joystick for steering
         */
        // NOTE: the left joystick goes negative when pushed upwards
        gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        gamePad1RightStickX = opMode.gamepad1.right_stick_x;

        motorPowerLF = 0;
        motorPowerLB = 0;
        motorPowerRF = 0;
        motorPowerRB = 0;

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
                (Math.abs(gamePad1LeftStickX) < -GAME_STICK_DEAD_ZONE) )
        {
            motorPowerLF = -gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerLB = -gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerRF = gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerRB = gamePad1LeftStickY - gamePad1LeftStickX;

            Log.v("BOK","LF:" + String.format("%.2f", motorPowerLF) +
                    "LB: " + String.format("%.2f", motorPowerLB) +
                    "RF: " + String.format("%.2f", motorPowerRF) +
                    "RB: " + String.format("%.2f", motorPowerRB));
        }

        // Right joystick is for turning
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerLF = motorPowerLB = gamePad1RightStickX;
            motorPowerRF = motorPowerRB = gamePad1RightStickX;

            Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
                    "LB: " + String.format("%.2f", motorPowerLB) +
                    "RF: " + String.format("%.2f", motorPowerRF) +
                    "RB: " + String.format("%.2f", motorPowerRB));
        }

        robotWithMecanumWheels.setPowerToDTMotors(
                motorPowerLF,
                motorPowerLB,
                motorPowerRF,
                motorPowerRB);
    }

}

