package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 */
@Autonomous(name="BoK Auto Position", group="BoK")
//@Disabled
public class BoKAutoPositionOpMode extends BoKAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new BoKAutoPosition(); // use interface (polymorphism)
        super.runOpMode();
    }
}