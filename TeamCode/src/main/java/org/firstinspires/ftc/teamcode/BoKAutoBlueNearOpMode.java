package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses BoKMecanumDT and BoKAutoBlueNear objects
 */
@Autonomous(name="BoK Auto BLUE Near", group="BoKBlue")
@Disabled
public class BoKAutoBlueNearOpMode extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        boolean status = false;
        BoKHardwareBot robot = new BoKMecanumDT();
        BoKAuto opMode = null;

        /*
         * Initialize the drive train and the robot subsystems variables.
         * The initHardware() method of the hardware class does all the work here
         */
        if (BoKHardwareBot.BoKHardwareStatus.BOK_HARDWARE_FAILURE == robot.initHardware(this)) {
            telemetry.addData("Status", "Hardware NOT initialized");
            telemetry.update();
        }
        else {
            // Send telemetry message to update hardware status
            telemetry.addData("Status", "Hardware initialized");
            telemetry.update();

            opMode = new BoKAutoBlueNear(); // use interface (polymorphism)
            if (BoKAuto.BoKAutoStatus.BOK_AUTO_FAILURE ==
                    opMode.initSoftware(this, robot, BoKAuto.BoKAllianceColor.BOK_ALLIANCE_BLUE)) {
                telemetry.addData("Status", "Software NOT initialized");
                telemetry.update();
            } else {
                status = true; // Hardware and software initialized!
                // Send telemetry message to update status
                telemetry.addData("Status", "Initialization complete. READY!");
                telemetry.update();
            }
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run the autonomous operation, if hardware and software is initialized
        // else do nothing
        if (status) {
            opMode.runSoftware();
        }
    }
}