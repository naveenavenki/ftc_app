package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
@TeleOp(name="BoK TeleOp", group="BoK6WD")
public class BoKTeleop extends LinearOpMode {
    protected BoKHardwareBot robot;
    protected double shooterServoStartPos;

    protected double leftGamepad1 = 0;
    protected double rightGamepad1 = 0;
    //private static final double MAX_POS     =  1.0;     // Maximum rotational position
    //private static final double MIN_POS     =  0.0;     // Minimum rotational position
    //private int currentLeftServoPosition = 0;
    //private int currentRightServoPosition = 0;
    //private boolean rampUpLeft = true;
    //private boolean rampUpRight = true;
    private static final double INITIAL_POS_LEFT  = 0.85;
    private static final double FINAL_POS_LEFT    = 0.35;
    private static final double INITIAL_POS_RIGHT = 0.0;
    private static final double FINAL_POS_RIGHT  = 0.5;
    private static final double SERVO_POS        = 0.0;
    private double  positionLeft = INITIAL_POS_LEFT;//(MAX_POS - MIN_POS) / 2; // Start at halfway position
    private double  positionRight = INITIAL_POS_RIGHT;//(MAX_POS - MIN_POS) / 2; // Start at halfway position
    private boolean SWEEPER_ON=false;

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

        robot.setShooterServoPos(SERVO_POS);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (SWEEPER_ON) {
                Log.v("BOK", "IS_BUSY="+robot.sweeperMotor.isBusy());
            }

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
                SWEEPER_ON = true;
                robot.setPowerToSweeper(1);
            }

            if(gamepad2.b)  {
                /*
                if (rampUpRight) {
                    positionRight += 0.1;
                    if (positionRight >= MAX_POS) {
                        positionRight = MAX_POS;
                        rampUpRight = !rampUpRight;   // Switch ramp direction
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
                if (positionRight == INITIAL_POS_RIGHT) {
                    positionRight = FINAL_POS_RIGHT;
                    robot.setRightPusherPos(positionRight);
                }
            }
            if(gamepad2.x) {
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
                if (positionLeft == INITIAL_POS_LEFT) {
                    positionLeft = FINAL_POS_LEFT;
                    robot.setLeftPusherPos(positionLeft);
                }
            }

            if(gamepad2.y){
                robot.setPowerToShooter(0);
            }
            if(gamepad2.a)
            {
                robot.setPowerToSweeper(0);
                SWEEPER_ON = false;
            }

            if(gamepad1.left_bumper){
                if(positionLeft != INITIAL_POS_LEFT) {
                    positionLeft = INITIAL_POS_LEFT;
                    robot.setLeftPusherPos(positionLeft);
                }
            }

            if(gamepad1.right_bumper){
                if(positionRight != INITIAL_POS_RIGHT) {
                    positionRight = INITIAL_POS_RIGHT;
                    robot.setRightPusherPos(positionRight);
                }
            
            }

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive(
        }
    }
}
