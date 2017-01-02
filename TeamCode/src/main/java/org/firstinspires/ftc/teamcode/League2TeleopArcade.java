package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Krishna Saxena on 11/26/2016.
 */

public class League2TeleopArcade {
    protected BoKHardwareBot robot;
    protected BoKAuto.BoKAlliance alliance;

    private double  positionLeft = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
    private double  positionRight = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;

    private double gamePad1RightStickX = 0;
    private double gamePad1LeftStickY = 0;
    private double rightPower = 0;
    private double leftPower = 0;
    private double liftPower = 0;
    private double driveDirection = 1.0;
    private double shooterMotorsSpeed = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL;

    private boolean shooterServosMinPos = false; // Shooter motors position
    private boolean shooterServosMidPos = true;

    private boolean goForBeacon = false;

    public void initSoftware(LinearOpMode opMode, BoKHardwareBot robot, BoKAuto.BoKAlliance redOrBlue) {
        // In teleop, do NOT use encoder
        robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the initial position of the button pushers (both pointed down)
        robot.pusherLeftServo.setPosition(positionLeft);
        robot.pusherRightServo.setPosition(positionRight);
        robot.liftServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_LIFT);

        // set the initial position of the shooter servo
        robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_TELEOP);

        // Send telemetry message to signify robot waiting;
        opMode.telemetry.addData("Status", "Software initialized");
        opMode.telemetry.update();
    }

    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException {
        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // NOTE: the left joystick goes negative when pushed upwards
            gamePad1LeftStickY = -opMode.gamepad1.left_stick_y;
            gamePad1RightStickX = opMode.gamepad1.right_stick_x;

            // Run wheels in tank mode
            if ((gamePad1LeftStickY < 0.05) && (gamePad1LeftStickY > -0.05)) {
                leftPower = 0;
                rightPower = 0;
            } else {
                leftPower = gamePad1LeftStickY*1.0;
                rightPower = gamePad1LeftStickY*1.0;
                if (!goForBeacon) {
                    if (leftPower < 0) {
                        leftPower = Range.clip(leftPower, -1.0, -0.3);
                    }
                    else {
                        leftPower = Range.clip(leftPower, 0.3, 1.0);
                    }
                    if (rightPower < 0) {
                        rightPower = Range.clip(rightPower, -1.0, -0.3);
                    }
                    else {
                        rightPower = Range.clip(rightPower, 0.3, 1.0);
                    }
                }
                else {
                    if (leftPower < 0) {
                        leftPower = leftPower*0.25;
                        //leftPower = Range.clip(leftPower, -0.15, -0.1);
                    }
                    else {
                        leftPower = leftPower*0.25;
                        //leftPower = Range.clip(leftPower, 0.2, 0.25);
                    }

                    if (rightPower < 0) {
                        rightPower = rightPower*0.25;
                        //rightPower = Range.clip(rightPower, -0.15, -0.1);
                    }
                    else {
                        rightPower = rightPower*0.25;
                        //rightPower = Range.clip(rightPower, 0.2, 0.25);
                    }

                }
            }


            if ((gamePad1RightStickX < 0.05) && (gamePad1RightStickX > -0.05)) {
                if (driveDirection == 1) {
                    robot.setPowerToMotors(leftPower, rightPower); // drive straight
                }
                else {
                    robot.setPowerToMotors(-rightPower, -leftPower); // drive backwards                }
                }
            } else if (gamePad1RightStickX > 0.05) { // direction is right
                // 1. We are throttling up and turning right (forward right)
                //    left power is +ve, right power is -ve
                // 2. We are throttling down and turning right (backward right)
                //    left power is -ve, right power is +ve
                //if (gamePad1LeftStickY > 0.05) { // we are throttling up and moving right
                rightPower = (-1 * gamePad1RightStickX) * rightPower; // left power is +ve, right power is -ve
                //}
                //else {  // we are throttling down: backward right
                //    rightPower = (-1 * gamePad1RightStickX) * rightPower; // left power is -ve, right power is +ve
                //}
                if (driveDirection == 1) {
                    robot.setPowerToMotors(leftPower, rightPower); // drive straight
                }
                else {
                    robot.setPowerToMotors(-rightPower, -leftPower); // drive backwards                }
                }
            } else { // direction is left (< -0.05)
                // 1. We are throttling up and turning left (forward left)
                //    left power is -ve (RightStickX is -ve), right power is +ve
                // 2. We are throttling down and turning left (backward left)
                //    left power is +ve (RightStickX is -ve), right power is -ve
                //if (gamePad1LeftStickY > 0.05) { // we are throttling up and moving left
                leftPower = gamePad1RightStickX * leftPower; // left power is -ve (RightStickX is -ve), right power is +ve
                //}
                //else { // we are throttling down, backward left
                //    leftPower = gamePad1RightStickX * leftPower; // left power is +ve (RightStickX is -ve), right power is -ve
                //}
                if (driveDirection == 1) {
                    robot.setPowerToMotors(leftPower, rightPower); // drive straight
                }
                else {
                    robot.setPowerToMotors(-rightPower, -leftPower); // drive backwards                }
                }
            }

            //telemetry.addData("Throttle:",  "%.2f" + " Direction %.2f", gamePad1LeftStickY, gamePad1RightStickX);
            //telemetry.addData("Power:", "left: %.2f" + " right: %.2f", leftPower, rightPower);
            //telemetry.update();

            //Log.v("BOK", "Throttle: " + String.format("%.2f", gamePad1LeftStickY) + " Direction: " + String.format("%.2f", gamePad1RightStickX));
            //Log.v("BOK", "Power: " + String.format("%.2f", leftPower) + ", " + String.format("%.2f", rightPower));

            if (opMode.gamepad2.right_bumper){
                robot.setPowerToShooter(shooterMotorsSpeed);
                robot.gateServo.setPosition(robot.FINAL_SERVO_POS_GATE);
            }

            if (opMode.gamepad2.left_bumper){
                robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_NORMAL);
            }

            if (opMode.gamepad2.right_stick_button)  {
                Log.v("BOK", "PositionRight: " + positionRight);

                if(positionRight != BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT) {
                    positionRight = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;
                    robot.pusherRightServo.setPosition(positionRight);
                }
            }

            if (opMode.gamepad2.left_stick_button) {
                Log.v("BOK", "PositionLeft: " + positionLeft);

                if(positionLeft != BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT) {
                    positionLeft = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
                    robot.pusherLeftServo.setPosition(positionLeft);
                }

            }

            if (opMode.gamepad2.y){
                robot.setPowerToShooter(0);
                robot.gateServo.setPosition(robot.INITIAL_SERVO_POS_GATE);
            }
            if (opMode.gamepad2.a)
            {
                robot.sweeperMotor.setPower(0);
            }

            if (opMode.gamepad2.x){
                if (positionLeft == BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT) {
                    positionLeft = BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT;
                    robot.pusherLeftServo.setPosition(positionLeft);
                }
            }

            if (opMode.gamepad2.b){
                if (positionRight == BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT) {
                    positionRight = BoKHardwareBot.FINAL_SERVO_POS_PUSHER_RIGHT;
                    robot.pusherRightServo.setPosition(positionRight);
                }
            }
            if (opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right){
                robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_REVERSE);
            }

            if (opMode.gamepad2.dpad_down){
                if(!shooterServosMinPos){
                    shooterServosMinPos=true;
                    shooterServosMidPos=false;
                    robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_TELEOP-0.1);
                }

            }
            if (opMode.gamepad2.dpad_up){
                if(!shooterServosMidPos){
                    shooterServosMidPos=true;
                    shooterServosMinPos=false;
                    robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_TELEOP);
                }
            }

            if (opMode.gamepad1.a) {
                if (!goForBeacon)
                    goForBeacon = true;
            }
            if (opMode.gamepad1.y) {
                if (goForBeacon)
                    goForBeacon = false;
            }
            if (opMode.gamepad2.right_trigger > 0.2){
                liftPower = 1.0;
            }
            else if(opMode.gamepad2.left_trigger > 0.2) {
                liftPower = -1.0;
            }
            else {
                liftPower = 0;
            }
            if (opMode.gamepad1.x) {
                robot.liftServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_LIFT);
                driveDirection = -1.0;
            }
            if (opMode.gamepad1.b) {
                driveDirection = 1.0;
            }
            if (opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right) {
                robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_REVERSE);
            }
            if (opMode.gamepad1.left_bumper) {
                shooterMotorsSpeed = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL;
            }
            if (opMode.gamepad1.right_bumper) {
                shooterMotorsSpeed = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL + 0.1;
            }
            robot.liftMotor.setPower(liftPower);

            opMode.telemetry.addData("Shooter Angle", robot.shooterServo.getPosition());
            opMode.telemetry.addData("Shooter Motors", shooterMotorsSpeed);
            opMode.telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second
            robot.waitForTick(BoKTeleop.METRONOME_TICK);
            opMode.idle(); // Always call idle() at the bottom of your while(opModeIsActive(
        }
    }
}