package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MM_TeleOp", group = "test")
public class MM_TeleOp extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    public void runOpMode() {
        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.driveWithSticks();
            robot.collector.controlFlywheels();
            robot.drivetrain.controlFoundation();
            robot.arm.toggleGripper();
            robot.collector.alignStone();
            robot.arm.rotateGripperWrist();
            robot.arm.armMove();
        }
    }
}