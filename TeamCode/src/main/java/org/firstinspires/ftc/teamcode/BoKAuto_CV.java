package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
@Autonomous(name="BoK Auto", group="BoK6WD")
public class BoKAuto_CV extends BoKAuto{


    @Override
    public void runOpMode() throws InterruptedException {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        BoKOpenCVTest Test = new BoKOpenCVTest();
        Test.initTest(this,null);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Software initialized");
        telemetry.update();


        //BoKAutoRunTest test = new BoKAutoRunTest();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run the autonomous operation

        Test.runTest(this,null);

    }
}
