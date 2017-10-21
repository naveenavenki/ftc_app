package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by urmanKrishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    private BoKGlyphArm arm;
    private static final double GAME_STICK_DEAD_ZONE      = 0.05;
    private static final double GAME_TRIGGER_DEAD_ZONE      = 0.1;
    private static final double   COUNTS_PER_MOTOR_REV    = 1120;

    public int tTAngle = 1120;
    private int tTAngleMax = 1120;
    private int tTAngleMin = 0;

    private boolean tank = false;

    private double speedCoef = 0.5;

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
            if(opMode.gamepad1.x) {
                tank = false;
            }
            if(opMode.gamepad1.b) {
                tank = true;
            }
            if(!tank) {
                moveRobot(opMode, robot);
            }
            else {
                moveRobotTank(opMode, robot);
            }

            /*
             * Gamepad 1 controls
             */
            if(opMode.gamepad1.y)
            {
                speedCoef = 0.2;
            }
            if(opMode.gamepad1.a)
            {
                speedCoef = 0.5;
            }
            /*
             * Gamepad 2 controls
             */

            if(opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) {

                robot.turnTable.setPower(opMode.gamepad2.right_trigger / 2);

                /*if ((tTAngle++) >= tTAngleMax) {
                    tTAngle = tTAngleMax;
                }
                else {
                    tTAngle++;
                }
                robot.turnTable.setTargetPosition(tTAngle);
                robot.turnTable.setPower(0.3);
                opMode.telemetry.addData("tTAngle #:", tTAngle);
                opMode.telemetry.update();*/

            }
            else {

                robot.turnTable.setPower(0.0);
            }

            if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
                robot.turnTable.setPower(-opMode.gamepad2.left_trigger / 2);

                /*if ((tTAngle--) <= tTAngleMin) {
                    tTAngle = tTAngleMin;
                }
                else {
                    tTAngle--;
                }
                robot.turnTable.setTargetPosition(-tTAngle);
                robot.turnTable.setPower(0.3);
                opMode.telemetry.addData("tTAngle #:", tTAngle);
                opMode.telemetry.update();*/

            }
            else {

                robot.turnTable.setPower(0.0);
            }

            if (opMode.gamepad2.left_stick_y < -0.2) {
                robot.upperArm.setPower(0.3);
            }
            else if (opMode.gamepad2.left_stick_y > 0.2) {
                robot.upperArm.setPower(-0.3);
            }
            else {
                robot.upperArm.setPower(0);
                robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if(opMode.gamepad2.right_stick_y < -0.2){

                arm.increaseClawWristPos(opMode.gamepad2.right_stick_y);
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if(opMode.gamepad2.right_stick_y > 0.2){

                arm.decreaseClawWristPos(opMode.gamepad2.right_stick_y);
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

            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (robot.flickerTouch.getState() == true)
            {
                opMode.telemetry.addData("Digital Touch", "Is not pressed");
                opMode.telemetry.update();
                //Log.v("BOK", "Hi");
            }
            else
            {
                Log.v("BOK", "Hi");
                opMode.telemetry.addData("Digital Touch", "Is Pressed");
                opMode.telemetry.update();
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
