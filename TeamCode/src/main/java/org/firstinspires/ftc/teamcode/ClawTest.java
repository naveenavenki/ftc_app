package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Krishna Saxena on 10/23/2017.
 */
@TeleOp(name = "Claw", group = "test")
//@Disabled
public class ClawTest extends LinearOpMode {

    Servo relicArm;
    DcMotor spool;

    @Override
    public void runOpMode() throws InterruptedException {

        relicArm = hardwareMap.servo.get("ra");
        spool = hardwareMap.dcMotor.get("sp");
        double servoPos = 0.83;

        relicArm.setPosition(0.83);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                spool.setPower(0.15);
            }
            else if(gamepad1.b){
                spool.setPower(-0.15);
            }
            else if(gamepad1.y){
                spool.setPower(0);
            }

            if(gamepad1.dpad_up)
                servoPos += 0.01;
            else if(gamepad1.dpad_down)
                servoPos -= 0.01;
            relicArm.setPosition(servoPos);
            telemetry.addData("pos", servoPos);
            telemetry.update();
        }
    }
}
