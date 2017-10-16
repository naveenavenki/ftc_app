package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    private BoKGlyphArm arm;
    private static final double GAME_STICK_DEAD_ZONE      = 0.05;
    private int uaTarget = -1;

    public enum BoKTeleStatus
    {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }

    public BoKTeleStatus initSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        arm = new BoKGlyphArm(robot);
        // Reset the initial position of the servos

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware(LinearOpMode opMode, BoKHardwareBot robot)
    {
        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            moveRobot(opMode, robot);
            /*
             * Gamepad 1 controls
             */
            /*
             * Gamepad 2 controls
             */

            if(opMode.gamepad2.right_trigger > 0.1) {

                robot.turnTable.setPower(opMode.gamepad2.right_trigger);

            }
            else {

                robot.turnTable.setPower(0.0);

            }

            if (opMode.gamepad2.left_trigger > 0.1) {

                robot.turnTable.setPower(-opMode.gamepad2.left_trigger);

            }
            else {

                robot.turnTable.setPower(0.0);

            }
/*
            if (opMode.gamepad2.dpad_up) {

                robot.upperArm.setPower(0.2);
            }
            else if(opMode.gamepad2.dpad_down){

                robot.upperArm.setPower(-0.1);
            }
            else {

                robot.upperArm.setPower(0);
                robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
*/
            if (opMode.gamepad2.left_stick_y < -0.2) {
                if (uaTarget == -1) {
                    uaTarget = arm.moveUpperArm(30.0, 0.3);
                }
            }
            if (opMode.gamepad2.left_stick_y > 0.2) {
                if (uaTarget == -1) {
                    uaTarget = arm.moveUpperArm(-30.0, 0.3);
                }
            }
            if (uaTarget != -1) {
                int pos = robot.upperArm.getCurrentPosition();
                int diff = (uaTarget <= 0) ? Math.abs(uaTarget) - Math.abs(pos) :
                        uaTarget - pos;
                Log.v("BOK", pos + ", " + diff);
                if (!robot.upperArm.isBusy()) {
                    robot.upperArm.setPower(0);
                    robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    uaTarget = -1;
                }
            }
            if(opMode.gamepad2.right_stick_y < -0.2){

                arm.increaseClawWristPos();
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if(opMode.gamepad2.right_stick_y > 0.2){

                arm.decreaseClawWristPos();
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if(opMode.gamepad2.b){

                arm.setClawGrabOpen ();
                //Log.v("BOK","CLAWOPEN");

            }

            if(opMode.gamepad2.a){

                arm.setClawGrabClose();
                //Log.v("BOK","CLAWCLOSED");

            }

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
        // Right joystick is for turning
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE)) {
            motorPowerLF = motorPowerLB = gamePad1RightStickX;
            motorPowerRF = motorPowerRB = gamePad1RightStickX;

            Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
                    "LB: " + String.format("%.2f", motorPowerLB) +
                    "RF: " + String.format("%.2f", motorPowerRF) +
                    "RB: " + String.format("%.2f", motorPowerRB));
        }

        robot.setPowerToDTMotors(
                motorPowerLF,
                motorPowerLB,
                motorPowerRF,
                motorPowerRB);
    }
}
