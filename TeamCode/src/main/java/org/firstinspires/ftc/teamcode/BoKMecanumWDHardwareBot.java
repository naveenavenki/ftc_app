package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Krishna Saxena on 9/24/2016.
 * Extends BoKHardwareBot to implement the 6 Wheel Drive train with 4 DC Motors.
 */
public class BoKMecanumWDHardwareBot extends BoK4MotorsDTBot {

    /*
     * Set methods:
     * 1. set power to Mecanum wheel drive
     */
    public void setPowerToMecanumDTMotors(double leftF, double leftB, double rightF, double rightB) {
        leftBack.setPower(leftB);
        rightBack.setPower(rightB);
        leftFront.setPower(leftF);
        rightFront.setPower(rightF);
        currentOpMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

}
