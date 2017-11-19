package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import android.util.Log;

/**
 * Created by Krishna Saxena on 11/1/2017.
 */
@TeleOp(name="BOK SETUP", group= "BoKZ")
//@Disabled
public class BoKInitTeleOpMode extends LinearOpMode
{
    private static final double INIT_ANGLE = 16;
    private static final double UA_POWER = 0.2;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double UPPER_ARM_STICK_DEAD_ZONE = 0.2;
    private static final double UPPER_ARM_MOTOR_POWER_SLOW = 0.2;

    BoKHardwareBot robot = new BoKMecanumDT();

    public void runOpMode()
    {
        boolean writeOnce = false;
        boolean moveOnce = false;

        robot.initHardware(this);
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        waitForStart();
        
        while(opModeIsActive()) {
            // GAMEPAD 2 CONTROLS
            // Left Stick Y:         Upper Arm
            // A:                    Move Upper Arm INIT_ANGLE (encoder)
            // B:                    Reset moving upper arm
            // Left & Right Trigger: Move wrist position
            // Y:                    Save wrist position to file
            if (gamepad2.left_stick_y < -UPPER_ARM_STICK_DEAD_ZONE) {
                robot.upperArm.setPower(UPPER_ARM_MOTOR_POWER_SLOW);
            }
            else if (gamepad2.left_stick_y > UPPER_ARM_STICK_DEAD_ZONE) {
                robot.upperArm.setPower(-UPPER_ARM_MOTOR_POWER_SLOW);
            }
            else {
                robot.upperArm.setPower(0);
                robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (gamepad2.a && !moveOnce) {
                moveOnce = true;
                robot.glyphArm.moveUpperArm(INIT_ANGLE, UA_POWER);
            }

            if (gamepad2.b && moveOnce) {
                moveOnce = false;
            }

            if (gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
               robot.glyphArm.decreaseClawWristPos(gamepad2.left_trigger);
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if (gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) {
                robot.glyphArm.increaseClawWristPos(gamepad2.right_trigger);
                //Log.v("BOK", String.format("%f", opMode.gamepad2.right_stick_y) );
            }

            if (gamepad2.y && !writeOnce) {
                writeOnce = true;
                File file = AppUtil.getInstance().getSettingsFile("BoKArmCalibration.txt");
                ReadWriteFile.writeFile(file,
                       Double.toString(robot.glyphArm.clawWrist.getPosition()));
                Log.v("BOK", "Value written to file: " +
                       Double.toString(robot.glyphArm.clawWrist.getPosition()));
            }
            robot.waitForTick(BoKHardwareBot.WAIT_PERIOD);
        }
    }
}
