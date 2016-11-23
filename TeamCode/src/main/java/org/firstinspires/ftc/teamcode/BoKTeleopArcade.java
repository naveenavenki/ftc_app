package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    private boolean goForBeaconRight = false;
    private boolean goForBeaconLeft  = false;
    private boolean isStageInit = false;
    private int goForBeaconStage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BoK6WDHardwareBot();

        if (BoKHardwareBot.BoKStatus.BOK_FAILURE == robot.initHardware(this))
        {
            throw new InterruptedException("Hardware not initialized");
        }

        // In teleop, do NOT use encoder
        robot.setModeForMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the initial position of the button pushers (both pointed down)
        robot.pusherLeftServo.setPosition(positionLeft);
        robot.pusherRightServo.setPosition(positionRight);

        // set the initial position of the shooter servo
        robot.shooterServo.setPosition(BoKHardwareBot.INITIAL_SHOOTER_SERVO_POS_TELEOP);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");

        robot.initGyro(this);
        telemetry.addData("Gyro", robot.gyroSensor.getIntegratedZValue());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (!goForBeaconRight && !goForBeaconLeft) {
                // NOTE: the left joystick goes negative when pushed upwards
                gamePad1LeftStickY = -gamepad1.left_stick_y;
                gamePad1RightStickX = gamepad1.right_stick_x;

                // Run wheels in tank mode
                if ((gamePad1LeftStickY < 0.05) && (gamePad1LeftStickY > -0.05)) {
                    leftPower = 0;
                    rightPower = 0;
                } else {
                    leftPower = gamePad1LeftStickY;
                    rightPower = gamePad1LeftStickY;
                }

                if ((gamePad1RightStickX < 0.05) && (gamePad1RightStickX > -0.05)) {
                    robot.setPowerToMotors(leftPower, rightPower); // drive straight
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
                    robot.setPowerToMotors(leftPower, rightPower);
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
                    robot.setPowerToMotors(leftPower, rightPower);
                }
            }

            //telemetry.addData("Throttle:",  "%.2f" + " Direction %.2f", gamePad1LeftStickY, gamePad1RightStickX);
            //telemetry.addData("Power:", "left: %.2f" + " right: %.2f", leftPower, rightPower);
            //telemetry.update();

            //Log.v("BOK", "Throttle: " + String.format("%.2f", gamePad1LeftStickY) + " Direction: " + String.format("%.2f", gamePad1RightStickX));
            //Log.v("BOK", "Power: " + String.format("%.2f", leftPower) + ", " + String.format("%.2f", rightPower));

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

            if (gamepad1.x || goForBeaconLeft) { // for approaching the beacon from the left
                double current_alpha, distance, delta;
                // approach from the left of the white line
                if (!goForBeaconLeft) {
                    goForBeaconLeft = true;
                }
                switch (goForBeaconStage) {
                    case 0:
                        // go to white
                        if (!isStageInit) {
                            robot.setPowerToMotors(BoKAuto.LEFT_MOTOR_POWER/2, BoKAuto.RIGHT_MOTOR_POWER/2);
                            isStageInit = true;
                        }
                        current_alpha = robot.odsSensor.getLightDetected();
                        if (current_alpha >= BoKAutoCommon.WHITE_LINE) {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                            isStageInit = false;
                        }
                        break;
                    case 1:
                        // go to gray
                        if (!isStageInit) {
                            robot.setPowerToMotors(BoKAuto.LEFT_MOTOR_POWER/2, BoKAuto.RIGHT_MOTOR_POWER/2);
                            isStageInit = true;
                        }
                        current_alpha = robot.odsSensor.getLightDetected();
                        if (current_alpha <= (BoKAutoCommon.LINE_EDGE)) {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                            isStageInit = false;
                        }
                        break;
                    case 2:
                        // turn to line edge
                        if (!isStageInit) {
                            robot.setPowerToMotors(-BoKAuto.LEFT_MOTOR_POWER/2, BoKAuto.RIGHT_MOTOR_POWER/2);
                            isStageInit = true;
                        }
                        current_alpha = robot.odsSensor.getLightDetected();
                        if (current_alpha <= BoKAutoCommon.LINE_EDGE) {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                            isStageInit = false;
                        }
                        break;

                    case 3:
                        // proportional line follower
                        distance = robot.rangeSensor.cmUltrasonic();
                        if (distance > League1AutoRed.ROBOT_DISTANCE_FROM_WALL_FOR_BEACON) {
                            current_alpha = robot.odsSensor.getLightDetected();

                            delta = (float) (BoKAutoCommon.LINE_EDGE-current_alpha); // if you are on white, delta is negative, otherwise it is positive
                            if (delta > 0) {
                                leftPower  = BoKAutoCommon.LEFT_POWER_LINE_FOLLOW-(delta*2);
                                rightPower = BoKAutoCommon.RIGHT_POWER_LINE_FOLLOW;
                            }
                            else {
                                leftPower = BoKAutoCommon.LEFT_POWER_LINE_FOLLOW;
                                rightPower = BoKAutoCommon.RIGHT_POWER_LINE_FOLLOW + (delta/2);
                            }
                            robot.setPowerToMotors(leftPower, rightPower);
                        }
                        else {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                            isStageInit = false;
                        }
                        break;
                    case 4:
                        // backing up from the beacon
                        distance = robot.rangeSensor.cmUltrasonic();
                        if (distance < League1AutoRed.ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON) {
                            if (!isStageInit) {
                                robot.setPowerToMotors(-BoKAuto.LEFT_MOTOR_POWER / 2, -BoKAuto.RIGHT_MOTOR_POWER / 2);
                                isStageInit = true;
                            }
                        }
                        else {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                        }
                        break;
                    default:
                        break;
                } // switch (goForBeaconStage)
            }

            if (gamepad1.b || goForBeaconRight) { // approaching the beacon from the right
                double current_alpha, distance, delta;
                // approach from the left of the white line
                if (!goForBeaconRight) {
                    goForBeaconRight = true;
                }
                switch (goForBeaconStage) {
                    case 0:
                        // go to white
                        if (!isStageInit) {
                            robot.setPowerToMotors(BoKAuto.LEFT_MOTOR_POWER/2, BoKAuto.RIGHT_MOTOR_POWER/2);
                            isStageInit = true;
                        }
                        current_alpha = robot.odsSensor.getLightDetected();
                        if (current_alpha >= BoKAutoCommon.WHITE_LINE) {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                            isStageInit = false;
                        }
                        break;
                    case 1:
                        // go to gray
                        if (!isStageInit) {
                            robot.setPowerToMotors(BoKAuto.LEFT_MOTOR_POWER/2, BoKAuto.RIGHT_MOTOR_POWER/2);
                            isStageInit = true;
                        }
                        current_alpha = robot.odsSensor.getLightDetected();
                        if (current_alpha <= (BoKAutoCommon.LINE_EDGE)) {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                            isStageInit = false;
                        }
                        break;
                    case 2:
                        // turn to line edge
                        if (!isStageInit) {
                            robot.setPowerToMotors(BoKAuto.LEFT_MOTOR_POWER/2, -BoKAuto.RIGHT_MOTOR_POWER/2);
                            isStageInit = true;
                        }
                        current_alpha = robot.odsSensor.getLightDetected();
                        if (current_alpha > BoKAutoCommon.LINE_EDGE) {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                            isStageInit = false;
                        }
                        break;
                    case 3:
                        // proportional line follower
                        distance = robot.rangeSensor.cmUltrasonic();
                        if (distance > League1AutoRed.ROBOT_DISTANCE_FROM_WALL_FOR_BEACON) {
                            current_alpha = robot.odsSensor.getLightDetected();

                            delta = (float) (current_alpha-BoKAutoCommon.LINE_EDGE); // if you are on white, delta is negative, otherwise it is positive
                            if (delta > 0) {
                                leftPower  = BoKAutoCommon.LEFT_POWER_LINE_FOLLOW-(delta/2);
                                rightPower = BoKAutoCommon.RIGHT_POWER_LINE_FOLLOW;
                            }
                            else {
                                leftPower = BoKAutoCommon.LEFT_POWER_LINE_FOLLOW;
                                rightPower = BoKAutoCommon.RIGHT_POWER_LINE_FOLLOW + (delta*2);
                            }
                            robot.setPowerToMotors(leftPower, rightPower);
                            //Log.v("BOK", "d: " + distance + ", a: " + current_alpha + ", l: " + leftPower + ", "+ rightPower );
                        }
                        else {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                            isStageInit = false;
                        }
                        break;
                    case 4:
                        // backing up from the beacon
                        distance = robot.rangeSensor.cmUltrasonic();
                        if (distance < League1AutoRed.ROBOT_DISTANCE_FROM_WALL_AFTER_BEACON) {
                            if (!isStageInit) {
                                robot.setPowerToMotors(-BoKAuto.LEFT_MOTOR_POWER / 2, -BoKAuto.RIGHT_MOTOR_POWER / 2);
                                isStageInit = true;
                            }
                        }
                        else {
                            robot.setPowerToMotors(0, 0);
                            goForBeaconStage++;
                        }
                        break;
                    default:
                        break;
                } // switch (goForBeaconStage)
            }

            if (gamepad1.y) {
                if (goForBeaconRight || goForBeaconLeft) {
                    robot.setPowerToMotors(0, 0);
                    goForBeaconLeft = goForBeaconRight = isStageInit = false;
                    goForBeaconStage = 0;
                }
            }

            telemetry.addData("Gyro: ", robot.gyroSensor.getIntegratedZValue());
            telemetry.addData("Distance: ", robot.rangeSensor.cmUltrasonic());
            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second
            robot.waitForTick(BoKAuto.METRONOME_TICK);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive(
        }
    }
}
