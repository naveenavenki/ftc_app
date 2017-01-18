package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Krishna Saxena on 11/26/2016.
 */

public class LeagueTeleopArcade implements BoKTeleop {
    protected BoKHardwareBot robot;
    protected BoKAuto.BoKAlliance alliance;

    private double posLeftPusher  = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
    private double posRightPusher = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;


    private double gamePad1RightStickX = 0;
    private double gamePad1LeftStickY = 0;

    private double rightDTPower = 0;
    private double leftDTPower = 0;

    private double capLiftPower = 0;
    private double driveDirection = 1.0;
    private double shooterMotorsSpeed = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL;

    private boolean shooterServosMinPos = false; // Shooter motors position
    private boolean shooterServosMidPos = true;

    private boolean goForBeacon = false;

    public void initSoftware(LinearOpMode opMode,
                             BoKHardwareBot robot, BoKAuto.BoKAlliance redOrBlue) {
        // In teleop, do NOT use encoder
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the initial position of the button pushers (both pointed down)
        robot.pusherLeftServo.setPosition(posLeftPusher);
        robot.pusherRightServo.setPosition(posRightPusher);
        robot.clawLockServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_CAP_CLAW);
        robot.partLiftGateServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PART_GATE);

        // set the initial position of the shooter servo
        robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_TELEOP);

        // Send telemetry message to signify robot waiting;
        opMode.telemetry.addData("Status", "Software initialized");
        opMode.telemetry.update();
    }

    public void runSoftware(LinearOpMode opMode, BoKHardwareBot robot) throws InterruptedException {
        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            /*
             * Gamepad1: Driver 1 controls the robot using the left joystick for throttle and
             * the right joystick for steering
             */
            // NOTE: the left joystick goes negative when pushed upwards
            gamePad1LeftStickY = -opMode.gamepad1.left_stick_y;
            gamePad1RightStickX = opMode.gamepad1.right_stick_x;

            // Run wheels in tank mode

            // Left joystick is for throttle
            if ((gamePad1LeftStickY < GAME_STICK_DEAD_ZONE) &&
                    (gamePad1LeftStickY > -GAME_STICK_DEAD_ZONE)) {
                leftDTPower = 0;
                rightDTPower = 0;
            } else {
                leftDTPower = gamePad1LeftStickY;
                rightDTPower = gamePad1LeftStickY;
                if (!goForBeacon) {
                    if (leftDTPower < 0) {
                        leftDTPower = Range.clip(leftDTPower,
                                -DT_POWER_HIGH_THRESHOLD, -DT_POWER_LOW_THRESHOLD);
                    }
                    else {
                        leftDTPower = Range.clip(leftDTPower,
                                DT_POWER_LOW_THRESHOLD, DT_POWER_HIGH_THRESHOLD);
                    }
                    if (rightDTPower < 0) {
                        rightDTPower = Range.clip(rightDTPower,
                                -DT_POWER_HIGH_THRESHOLD, -DT_POWER_LOW_THRESHOLD);
                    }
                    else {
                        rightDTPower = Range.clip(rightDTPower,
                                DT_POWER_LOW_THRESHOLD, DT_POWER_HIGH_THRESHOLD);
                    }
                }
                else { // going for beacon (or with cap-ball lift): SLOW DOWN
                    leftDTPower = leftDTPower * DT_POWER_REDUCTION_FACTOR;
                    rightDTPower = rightDTPower * DT_POWER_REDUCTION_FACTOR;
                }
            }

            // Right joystick is for steering
            if ((gamePad1RightStickX < GAME_STICK_DEAD_ZONE) &&
                    (gamePad1RightStickX > -GAME_STICK_DEAD_ZONE)) {
                if (driveDirection == 1) {
                    robot.setPowerToDTMotors(leftDTPower, rightDTPower);  // drive straight
                }
                else {
                    // drive straight backwards
                    robot.setPowerToDTMotors(-rightDTPower, -leftDTPower);
                }
            } else if (gamePad1RightStickX > GAME_STICK_DEAD_ZONE) { // direction is right
                // left power is +ve, right power is -ve
                rightDTPower = (-1 * gamePad1RightStickX) * rightDTPower;
                if (driveDirection == 1) {
                    robot.setPowerToDTMotors(leftDTPower, rightDTPower);   // turn right
                }
                else {
                    // turn right backwards by swapping left and right power
                    robot.setPowerToDTMotors(-rightDTPower, -leftDTPower);
                }
            } else { // direction is left (< -0.05)
                // left power is -ve (RightStickX is -ve), right power is +ve
                leftDTPower = gamePad1RightStickX * leftDTPower;
                if (driveDirection == 1) {
                    robot.setPowerToDTMotors(leftDTPower, rightDTPower); // turn left
                }
                else {
                    // turn left backwards by swapping left and right power
                    robot.setPowerToDTMotors(-rightDTPower, -leftDTPower);
                }
            }

            //telemetry.addData("Throttle:",  "%.2f" + " Direction %.2f",
            // gamePad1LeftStickY, gamePad1RightStickX);
            //telemetry.addData("Power:", "left: %.2f" + " right: %.2f", leftDTPower, rightDTPower);
            //telemetry.update();

            /*
             * Game pad 2 controls
             * right bumper: turn ON shooter motors and open the particle lift gate
             * left_bumper: turn ON sweeper motor
             * right stick button: retract right beacon pusher
             * left stick button: retract left beacon pusher
             * Y: turn OFF shooter motors and close the particle lift gate
             * A: turn OFF sweeper motor
             * X: extend left beacon pusher
             * B: extend right beacon pusher
             * dpad_left/right: reverse sweeper motor
             * dpad_down: set shooter servo down (when close to the center vortex)
             * dpad_up: raise shooter servo up
             * right_trigger: raise the cap-ball lift (must keep it pressed)
             * left_trigger: lower the cap-ball lift (must keep it pressed)
            */
            if (opMode.gamepad2.right_bumper){
                robot.setPowerToShooterMotors(shooterMotorsSpeed);
                robot.partLiftGateServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_PART_GATE);
            }

            if (opMode.gamepad2.left_bumper){
                robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_NORMAL);
            }

            if (opMode.gamepad2.right_stick_button)  {
                Log.v("BOK", "PositionRight: " + posRightPusher);

                if(posRightPusher != BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT) {
                    posRightPusher = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;
                    robot.pusherRightServo.setPosition(posRightPusher);
                }
            }

            if (opMode.gamepad2.left_stick_button) {
                Log.v("BOK", "PositionLeft: " + posLeftPusher);

                if(posLeftPusher != BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT) {
                    posLeftPusher = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
                    robot.pusherLeftServo.setPosition(posLeftPusher);
                }

            }

            if (opMode.gamepad2.y){
                robot.setPowerToShooterMotors(0);
                robot.partLiftGateServo.setPosition(BoKHardwareBot.INITIAL_SERVO_POS_PART_GATE);
            }

            if (opMode.gamepad2.a)
            {
                robot.sweeperMotor.setPower(0);
            }

            if (opMode.gamepad2.x){
                if (posLeftPusher == BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT) {
                    posLeftPusher = BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT;
                    robot.pusherLeftServo.setPosition(posLeftPusher);
                }
            }

            if (opMode.gamepad2.b){
                if (posRightPusher == BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT) {
                    posRightPusher = BoKHardwareBot.FINAL_SERVO_POS_PUSHER_RIGHT;
                    robot.pusherRightServo.setPosition(posRightPusher);
                }
            }

            /*
             * Game pad 2 controls (continued)
             * dpad_left/right: reverse sweeper motor
             * dpad_down: set shooter servo down (when close to the center vortex)
             * dpad_up: raise shooter servo up
             * right_trigger: raise the cap-ball lift (must keep it pressed)
             * left_trigger: lower the cap-ball lift (must keep it pressed)
             */
            if (opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right){
                robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_REVERSE);
            }

            if (opMode.gamepad2.dpad_down){
                if(!shooterServosMinPos){
                    shooterServosMinPos=true;
                    shooterServosMidPos=false;
                    robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_TELEOP-
                            SHOOTER_ANGLE_REDUCTION);
                }

            }
            if (opMode.gamepad2.dpad_up){
                if(!shooterServosMidPos){
                    shooterServosMidPos=true;
                    shooterServosMinPos=false;
                    robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_TELEOP);
                }
            }

            if (opMode.gamepad2.right_trigger > CAPBALL_LIFT_DEAD_ZONE){
                capLiftPower = CAPBALL_LIFT_POWER;
            }
            else if(opMode.gamepad2.left_trigger > CAPBALL_LIFT_DEAD_ZONE) {
                capLiftPower = -CAPBALL_LIFT_POWER;
            }
            else {
                capLiftPower = 0;
            }
            if (opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right) {
                robot.sweeperMotor.setPower(BoKHardwareBot.SWEEPER_MOTOR_POWER_REVERSE);
            }

            /*
            * Game pad 1 controls
            * Left Joystick: Throttle
            * Right Joystick: Turn
            * A: go slow (for Beacon pusher or when handling cap ball
            * Y: go nornal
            * X: release claw lock, reverse the drive (front is back, back is front)
            * B: drive normal
            * left bumper: set shooter motors power normal
            * right bumper: set shooter motors power + 0.1
            */
            if (opMode.gamepad1.a) {
                if (!goForBeacon)
                    goForBeacon = true;
            }
            if (opMode.gamepad1.y) {
                if (goForBeacon)
                    goForBeacon = false;
            }

            if (opMode.gamepad1.x) {
                robot.clawLockServo.setPosition(BoKHardwareBot.FINAL_SERVO_POS_CAP_CLAW);
                driveDirection = -1.0;
            }
            if (opMode.gamepad1.b) {
                driveDirection = 1.0;
            }

            if (opMode.gamepad1.left_bumper) {
                shooterMotorsSpeed = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL;
            }
            if (opMode.gamepad1.right_bumper) {
                shooterMotorsSpeed = BoKHardwareBot.SHOOTER_MOTORS_POWER_NORMAL + 0.1;
            }


            robot.capLiftMotor.setPower(capLiftPower);

            opMode.telemetry.addData("Shooter Angle", robot.shooterServo.getPosition());
            opMode.telemetry.addData("Shooter Motors", shooterMotorsSpeed);
            opMode.telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second
            robot.waitForTick(METRONOME_TICK);
            opMode.idle(); // Always call idle() at the bottom of your while(opModeIsActive(
        }
    }
}