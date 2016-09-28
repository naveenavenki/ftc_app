package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
@Autonomous(name="BoK Auto Color", group="BoK6WD")
public class BoKAuto extends LinearOpMode{
    protected BoKHardwareBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BoK6WDHardwareBot();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        if (BoKHardwareBot.BokStatus.BOK_FAILURE == robot.init(this)) {
            throw new InterruptedException("Hardware not initialized!");
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run the autonomous operation
        BoKAutoColorTest test = new BoKAutoColorTest();
        test.runTest(this, robot);
    }
}
