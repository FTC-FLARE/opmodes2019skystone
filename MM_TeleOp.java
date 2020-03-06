package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MM_TeleOp", group = "Mechanical Meltdown")
public class MM_TeleOp extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    public void runOpMode() {
        robot.init();
        robot.drivetrain.runWithoutEncoder();
        robot.drivetrain.unbrake();

        telemetry.addLine("Press Play to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controlArm();
            controlCollector();
            controlDrivetrain();
            telemetry.update();
        }
    }

    public void controlDrivetrain(){
        robot.drivetrain.driveWithSticks();
        robot.drivetrain.controlFoundation();
    }

    public void controlArm() {
        robot.arm.toggleGripper();
        robot.arm.rotateGripperWrist();
        robot.arm.armMove();
        robot.arm.deployCapstone();
    }

    public void controlCollector(){
        robot.collector.alignStone();
        robot.collector.controlFlywheels();
        robot.collector.moveBlueSkystick();
        robot.collector.moveRedSkystick();
        robot.teleOpAutoCollect();
    }
}