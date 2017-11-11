package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It extends BoKTeleOpMode but sets up GamePad2 trigger control differently.
 */
@TeleOp(name="BoK Teleop 2", group="BoKTele")
public class BoKTeleOpMode2 extends BoKTeleOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Use left trigger to increase wrist position
        trigger_left_decrease = false;
        super.runOpMode();
    }
}