package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "MM_AutoTest", group = "test")
public class MM_Auto extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    public void runOpMode() {
        robot.init();
        robot.vuforia.init();

        waitForStart();

        robot.vuforia.activateTarget();

        robot.drivetrain.resetEncoder();
        robot.drivetrain.driveWithInches(-19,.20);
//        robot.driveToSkystone();
//        robot.collector.skystickDown();
        robot.drivetrain.driveWithInches(20,.5);
        robot.drivetrain.gyroTurn(.25,90);
        robot.drivetrain.driveWithInches(36,.5);
//        robot.collector.skystickUp();
    }
}
