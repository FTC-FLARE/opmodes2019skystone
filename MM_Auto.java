package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "MM_AutoTest", group = "test")
public class MM_Auto extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    public void runOpMode() {
        robot.init();
        robot.drivetrain.init();

        waitForStart();

        robot.drivetrain.driveWithInches(10,.5);
        sleep(1000);
        robot.drivetrain.driveWithInches(15,.25);
    }
}
