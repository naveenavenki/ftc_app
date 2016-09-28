package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by krish_000 on 9/24/2016.
 */
@TeleOp(name="BoK TeleOp", group="BoK6WD")
public class BoKTeleop extends LinearOpMode {
    protected BoKHardwareBot robot;

    protected double leftGamepad1 = 0;
    protected double rightGamepad1 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BoK6WDHardwareBot();
        //robot = new BoK4WDHardwareBot();

        if (BoKHardwareBot.BoKStatus.BOK_FAILURE == robot.initHardware(this))
        {
            throw new InterruptedException("Hardware not initialized");
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed
            leftGamepad1 = gamepad1.left_stick_y;
            rightGamepad1 = gamepad1.right_stick_y;

            robot.setPowerToMotors(leftGamepad1, rightGamepad1);

            telemetry.addData("left",  "%.2f", leftGamepad1);
            telemetry.addData("right", "%.2f", rightGamepad1);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive(
        }
    }
}
