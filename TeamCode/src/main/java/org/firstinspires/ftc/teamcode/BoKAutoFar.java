package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 9/24/2016.
 * Registers the opMode with the driver station.
 * It uses BoK6WDHardwareBot and LeagueAutoBlueBeacon objects.
 */
@Autonomous(name="BoK Auto Far", group="BoK6WD")
//@Disabled
public class BoKAutoFar extends LinearOpMode
{
    protected BoKHardwareBot robot;

    @Override
    public void runOpMode()  {
        robot = new BoK6WDHardwareBot();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        if (BoKHardwareBot.BoKStatus.BOK_FAILURE == robot.initHardware(this)) {
            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Hardware NOT initialized");
            telemetry.update();
            return;
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        BoKAuto test = new LeagueAutoFar(); // use interface (polymorphism)
        test.initSoftware(this, robot, BoKAuto.BoKAlliance.BOK_ALLIANCE_BLUE);

        telemetry.addData("Status", "Software initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run the autonomous operation
        test.runSoftware(this, robot);
    }
}
