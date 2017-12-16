package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by shiv on 12/5/2017.
 */
@TeleOp(name="James", group = "Bok")
@Disabled
public class JamesTeleOp extends LinearOpMode {

    CRServo lf,rf,lb,rb;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.crservo.get("lf");
        rf = hardwareMap.crservo.get("rf");
        lb = hardwareMap.crservo.get("lb");
        rb = hardwareMap.crservo.get("rb");

        waitForStart();
        while (opModeIsActive()){
            double bp = gamepad1.left_stick_y;
            double fp = gamepad1.right_stick_y;
            lf.setPower(bp);
            rf.setPower(-bp);
            lb.setPower(fp);
            rb.setPower(-fp);

        }
    }
}
