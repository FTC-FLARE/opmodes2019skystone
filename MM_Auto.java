package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "MM_AutoTest", group = "test")
public class MM_Auto extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    public void runOpMode() {
        robot.init();

        waitForStart();

        robot.drivetrain.driveWithInches(12.5,.5);
        robot.drivetrain.runWithEncoder();
        robot.drivetrain.gyroTurn(.5,-45);
        robot.drivetrain.runToPosition();
        robot.drivetrain.driveWithInches(5,.5);
        robot.drivetrain.runWithEncoder();
        robot.drivetrain.gyroTurn(.5,-270);
        robot.drivetrain.runToPosition();
        robot.drivetrain.driveWithInches(25,.5);
        robot.drivetrain.runWithEncoder();
        robot.drivetrain.gyroTurn(.5,-179);
        robot.drivetrain.runToPosition();
        robot.drivetrain.driveWithInches(30,.5);
    }
}