package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
//@TeleOp(name="BoK TeleOp", group="BoK6WD")
public class BoKTeleop extends LinearOpMode {
    protected BoKHardwareBot robot;
    protected double shooterServoStartPos;

    protected double leftGamepad1 = 0;
    protected double rightGamepad1 = 0;
    private static final double SHOOTER_SERVO_POS   = 0.1;
    private static final double SWEEPER_MOTOR_POWER = 0.9;
    private double  positionLeft = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
    private double  positionRight = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;

    private static final double MAX_POS     =  1.0;     // Maximum rotational position
    private static final double MIN_POS     =  0.0;     // Minimum rotational position
    private int currentLeftServoPosition = 0;
    private int currentRightServoPosition = 0;
    private boolean rampUpLeft = true;
    private boolean rampUpRight = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BoK6WDHardwareBot();
        //robot = new BoK4WDHardwareBot();

        if (BoKHardwareBot.BoKStatus.BOK_FAILURE == robot.initHardware(this))
        {
            throw new InterruptedException("Hardware not initialized");
        }

        // set the initial position (both pointed down)
        robot.setLeftPusherPos(positionLeft);
        robot.setRightPusherPos(positionRight);

        robot.setShooterServoPos(SHOOTER_SERVO_POS);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed
            leftGamepad1 = -gamepad1.left_stick_y;
            rightGamepad1 = -gamepad1.right_stick_y;

            robot.setPowerToMotors(leftGamepad1, rightGamepad1);

            telemetry.addData("left",  "%.2f", leftGamepad1);
            telemetry.addData("right", "%.2f", rightGamepad1);
            telemetry.update();

            if(gamepad2.right_bumper){
                robot.setPowerToShooter(1);
            }

            if(gamepad2.left_bumper){
                robot.setPowerToSweeper(SWEEPER_MOTOR_POWER);
            }

            if(gamepad1.right_bumper)  {
    /*
                if (rampUpRight) {
                    positionRight += 0.1;
                    if (positionRight >= MAX_POS) {
                        positionRight = MAX_POS;
                        rampUpRight = !rampUpRight;
                        // Switch ramp direction

                    }
                }
                else {

                    positionRight -= 0.1;
                    if (positionRight <= MIN_POS) {
                        positionRight = MIN_POS;
                        rampUpRight = !rampUpRight;   // Switch ramp direction
                    }
                }

                Log.v("BOK", "PositionRight: " + positionRight);
                robot.setRightPusherPos(positionRight);
                sleep(50);
                idle();
                */

                Log.v("BOK", "PositionRight: " + positionRight);

                if(positionRight != BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT) {
                    positionRight = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT;
                    robot.setRightPusherPos(positionRight);
                }

            }
            if(gamepad1.left_bumper) {
/*
                if (rampUpLeft) {
                    positionLeft += 0.1;
                    if (positionLeft >= MAX_POS) {
                        positionLeft = MAX_POS;
                        rampUpLeft = !rampUpLeft;   // Switch ramp direction
                    }
                }
                else {

                    positionLeft -= 0.1;
                    if (positionLeft <= MIN_POS) {
                        positionLeft = MIN_POS;
                        rampUpLeft = !rampUpLeft;   // Switch ramp direction
                    }
                }

                Log.v("BOK", "PositionLeft: " + positionLeft);
                robot.setLeftPusherPos(positionLeft);
                sleep(50);
                idle();
                */

                Log.v("BOK", "PositionLeft: " + positionLeft);

                if(positionLeft != BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT) {
                    positionLeft = BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT;
                    robot.setLeftPusherPos(positionLeft);
                }

            }

            if(gamepad2.y){
                robot.setPowerToShooter(0);
            }
            if(gamepad2.a)
            {
                robot.setPowerToSweeper(0);
            }

            if(gamepad2.x){
                if (positionLeft == BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_LEFT) {
                    positionLeft = BoKHardwareBot.FINAL_SERVO_POS_PUSHER_LEFT;
                    robot.setLeftPusherPos(positionLeft);
                }
            }

            if(gamepad2.b){
                if (positionRight == BoKHardwareBot.INITIAL_SERVO_POS_PUSHER_RIGHT) {
                    positionRight = BoKHardwareBot.FINAL_SERVO_POS_PUSHER_RIGHT;
                    robot.setRightPusherPos(positionRight);
                }
            }
            if(gamepad1.a){
                robot.setPowerToSweeper(-SWEEPER_MOTOR_POWER);
            }

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive(
        }
    }
}
