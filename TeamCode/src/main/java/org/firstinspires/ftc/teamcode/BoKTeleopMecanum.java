package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by shiv on 2/4/2017.
 */



/**
 * Created by Krishna Saxena on 9/24/2016.
 * Registers the opMode with the driver station.
 * It uses BoK6WDHardwareBot and LeagueAutoRedBeacon objects.
 */
@TeleOp(name="BoK TeleOpMecanum", group="BoKMecanum")
public class BoKTeleopMecanum extends LinearOpMode {
    protected BoKHardwareBot robot;

    @Override
    public void runOpMode()  {
        robot = new BoK6WDHardwareBot();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        if (BoKHardwareBot.BoKStatus.BOK_FAILURE == robot.initHardware(this))
        {
            telemetry.addData("Status", "Hardware NOT initialized");
            telemetry.update();
            return;
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        BoKTeleop test = new MecanumTeleOp();
        test.initSoftware(this, robot, BoKAuto.BoKAlliance.BOK_ALLIANCE_RED);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        test.runSoftware(this, robot);
    }
}