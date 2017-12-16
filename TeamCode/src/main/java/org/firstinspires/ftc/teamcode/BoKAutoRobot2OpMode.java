package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses BoKMecanumDT and BoKAutoBlueFar objects
 */
@Autonomous(name="BoK Auto Robot2", group="BoKBlue")
@Disabled
public class BoKAutoRobot2OpMode extends BoKAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new BoKAutoRobot2(); // use interface (polymorphism)
        super.runOpMode();
    }
}