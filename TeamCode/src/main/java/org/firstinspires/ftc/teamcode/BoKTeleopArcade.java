package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.text.DecimalFormat;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
@TeleOp(name="BoK TeleOpArcade", group="BoK6WD")
public class BoKTeleopArcade extends LinearOpMode {
    protected BoKHardwareBot robot;

    private double  positionLeft = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
    private double  positionRight = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;

    private double gamePad1RightStickX = 0;
    private double gamePad1LeftStickY = 0;
    private double rightPower = 0;
    private double leftPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BoK6WDHardwareBot();

        if (BoKHardwareBot.BoKStatus.BOK_FAILURE == robot.initHardware(this))
        {
            throw new InterruptedException("Hardware not initialized");
        }

        // set the initial position of the button pushers (both pointed down)
        robot.pusherLeftServo.setPosition(positionLeft);
        robot.pusherRightServo.setPosition(positionRight);

        // set the initial position of the shooter servo
        robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // NOTE: the left joystick goes negative when pushed upwards
            gamePad1LeftStickY  = -gamepad1.left_stick_y;
            gamePad1RightStickX = gamepad1.right_stick_x;

            // Run wheels in tank mode
            if((gamePad1LeftStickY < 0.05) && (gamePad1LeftStickY > -0.05)) {
                leftPower = 0;
                rightPower = 0;
            }
            else {
                leftPower = gamePad1LeftStickY;
                rightPower = gamePad1LeftStickY;
            }

            if((gamePad1RightStickX < 0.05)&& (gamePad1RightStickX > -0.05)) {
                robot.setPowerToMotors(leftPower, rightPower); // drive straight
            }
            else if (gamePad1RightStickX > 0.05) { // direction is right
                // 1. We are throttling up and turning right (forward right)
                //    left power is +ve, right power is -ve
                // 2. We are throttling down and turning right (backward right)
                //    left power is -ve, right power is +ve
                //if (gamePad1LeftStickY > 0.05) { // we are throttling up and moving right
                    rightPower = (1-gamePad1RightStickX) * rightPower; // left power is +ve, right power is -ve
                //}
                //else {  // we are throttling down: backward right
                //    rightPower = (-1 * gamePad1RightStickX) * rightPower; // left power is -ve, right power is +ve
                //}
                robot.setPowerToMotors(leftPower, rightPower);
            }
            else { // direction is left (< -0.05)
                // 1. We are throttling up and turning left (forward left)
                //    left power is -ve (RightStickX is -ve), right power is +ve
                // 2. We are throttling down and turning left (backward left)
                //    left power is +ve (RightStickX is -ve), right power is -ve
                //if (gamePad1LeftStickY > 0.05) { // we are throttling up and moving left
                    leftPower = (1-Math.abs(gamePad1RightStickX)) * leftPower; // left power is -ve (RightStickX is -ve), right power is +ve
                //}
                //else { // we are throttling down, backward left
                //    leftPower = gamePad1RightStickX * leftPower; // left power is +ve (RightStickX is -ve), right power is -ve
                //}
                robot.setPowerToMotors(leftPower, rightPower);
            }

            telemetry.addData("Throttle:",  "%.2f" + " Direction %.2f", gamePad1LeftStickY, gamePad1RightStickX);
            telemetry.addData("Power:", "left: %.2f" + " right: %.2f", leftPower, rightPower);
            telemetry.update();

            Log.v("BOK", "Throttle: " + String.format("%.2f", gamePad1LeftStickY) + " Direction: " + String.format("%.2f", gamePad1RightStickX));
            Log.v("BOK", "Power: " + String.format("%.2f", leftPower) + ", " + String.format("%.2f", rightPower));

            if(gamepad2.right_bumper){
                robot.setPowerToShooter(1);
            }

            if(gamepad2.left_bumper){
                robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_NORMAL);
            }

            if(gamepad1.right_bumper)  {
                Log.v("BOK", "PositionRight: " + positionRight);

                if(positionRight != BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT) {
                    positionRight = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;
                    robot.pusherRightServo.setPosition(positionRight);
                }
            }

            if(gamepad1.left_bumper) {
                Log.v("BOK", "PositionLeft: " + positionLeft);

                if(positionLeft != BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT) {
                    positionLeft = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
                    robot.pusherLeftServo.setPosition(positionLeft);
                }

            }

            if(gamepad2.y){
                robot.setPowerToShooter(0);
            }
            if(gamepad2.a)
            {
                robot.sweeperMotor.setPower(0);
            }

            if(gamepad2.x){
                if (positionLeft == BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT) {
                    positionLeft = BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT;
                    robot.pusherLeftServo.setPosition(positionLeft);
                }
            }

            if(gamepad2.b){
                if (positionRight == BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT) {
                    positionRight = BoKHardwareBot.FINAL_SERVO_POS_PUSHER_RIGHT;
                    robot.pusherRightServo.setPosition(positionRight);
                }
            }
            if(gamepad1.a){
                robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_REVERSE);
            }

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second
            robot.waitForTick(BoKAuto.METRONOME_TICK);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive(
        }
    }
}
