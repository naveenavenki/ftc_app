package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by james on 10/14/2017.
 */
@TeleOp(name = "ArmTest", group = "TeleOp")
public class Robot extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        RobotBody robot = new RobotBody();
        if(robot.initDriveTrain(this) == -1) {
            telemetry.addData("Status", "Drivetrain not Initialized");
            telemetry.update();
            return;
        }
        telemetry.addData("Status", "Drivetrain Initialized");
        telemetry.update();

        if(robot.initRobot(this) == -1) {
            telemetry.addData("Status", "RobotBody not Initialized");
            telemetry.update();
            return;
        }
        telemetry.addData("Status", "RobotBody Initialized");
        telemetry.update();

        waitForStart();

        robot.turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(opModeIsActive()){
            robot.moveArm(this);
            //robot.moveDriveTrain(this);
        }
    }
}
