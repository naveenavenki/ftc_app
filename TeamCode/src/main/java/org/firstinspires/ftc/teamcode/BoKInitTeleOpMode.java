package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by shiv on 11/1/2017.
 */
@TeleOp(name="INITIALIZE", group= "BoKInit")
public class BoKInitTeleOpMode extends LinearOpMode {
    private static final double INIT_ANGLE = 15;
    private static final double UA_POWER = 0.1;

    BoKHardwareBot robot = new BoKMecanumDT();

    public void runOpMode() throws InterruptedException {
        robot.initHardware(this);
        robot.glyphArm.moveUpperArm(INIT_ANGLE, UA_POWER);
        waitForStart();
    }
}
