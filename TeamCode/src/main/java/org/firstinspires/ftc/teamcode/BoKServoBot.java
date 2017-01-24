package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by shiv on 11/23/2016.
 */

public class BoKServoBot extends LinearOpMode {
    Servo servo;
    @Override
    public void runOpMode() {
        servo = hardwareMap.servo.get("ps");

        waitForStart();

        servo.setPosition(0);
        telemetry.addData("Pos", servo.getPosition());
        telemetry.update();
        sleep(5000);
        servo.setPosition(1);
        telemetry.addData("Pos", servo.getPosition());
        telemetry.update();
        sleep(4000);
        servo.setPosition(0);
        telemetry.addData("Pos", servo.getPosition());
        telemetry.update();
        sleep(3000);
        servo.setPosition(1);
        telemetry.addData("Pos", servo.getPosition());
        telemetry.update();
        sleep(2000);
        servo.setPosition(0);
        telemetry.addData("Pos", servo.getPosition());
        telemetry.update();
        sleep(10000);
    }
}
