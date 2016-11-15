package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
@Autonomous(name="BoK Auto League", group="BoK6WD")
//@Disabled
public class BoKAutoRed extends LinearOpMode
{
    protected BoKHardwareBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BoK6WDHardwareBot();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        if (BoKHardwareBot.BoKStatus.BOK_FAILURE == robot.initHardware(this)) {
            throw new InterruptedException("Hardware not initialized!");
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        //BoKVuforiaTest test = new BoKVuforiaTest();
        League1Auto test = new League1Auto();
        test.initTest(this, robot);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run the autonomous operation
        test.runTest(this, robot);
    }
}
