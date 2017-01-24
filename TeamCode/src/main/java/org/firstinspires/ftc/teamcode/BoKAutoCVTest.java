package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
//@Autonomous(name="BoK Auto CV", group="BoK6WD")
public class BoKAutoCVTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        BoKOpenCVTest Test = new BoKOpenCVTest();
        Test.initSoftware(this, null, BoKAuto.BoKAlliance.BOK_ALLIANCE_RED);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Software initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run the autonomous operation

        Test.runSoftware(this, null);

    }
}
