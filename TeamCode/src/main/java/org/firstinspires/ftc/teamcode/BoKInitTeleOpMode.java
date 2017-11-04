package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by shiv on 11/1/2017.
 */
@TeleOp(name="BOK SETUP", group= "BoKZ")
//@Disabled
public class BoKInitTeleOpMode extends LinearOpMode {
    private static final double INIT_ANGLE = 15;
    private static final double UA_POWER = 0.2;

    BoKHardwareBot robot = new BoKMecanumDT();

    public void runOpMode() throws InterruptedException {
        robot.initHardware(this);
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        waitForStart();
        robot.glyphArm.moveUpperArm(INIT_ANGLE, UA_POWER);
    }
}
