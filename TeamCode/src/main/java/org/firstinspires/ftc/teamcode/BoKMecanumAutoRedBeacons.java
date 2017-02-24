package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 9/24/2016.
 * Registers the opMode with the driver station.
 * It uses BoK4MotorsDTBot and LeagueAutoRedBeacon objects.
 */
@Autonomous(name="BoK  Mecanum Auto RED Beacons", group="BoK6WD")
//@Disabled
public class BoKMecanumAutoRedBeacons extends LinearOpMode
{
    protected BoKHardwareBot robot;

    @Override
    public void runOpMode() {
        robot = new BoK4MotorsDTBot();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        if (BoKHardwareBot.BoKStatus.BOK_FAILURE == robot.initHardware(this)) {
            telemetry.addData("Status", "Hardware NOT initialized");
            telemetry.update();
            return;
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        BoKAuto test = new MecannumRedBeacons();  // use interface (polymorphism)
        test.initSoftware(this, robot, BoKAuto.BoKAlliance.BOK_ALLIANCE_RED);

        telemetry.addData("Status", "Software initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run the autonomous operation
        test.runSoftware(this, robot);
    }
}
