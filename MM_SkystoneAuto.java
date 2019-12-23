package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// naming is (alliance color, side of field, amount of skystones, any other auto functions)
@Autonomous(name = "MM_Skystone_Auto", group = "Mechanical Meltdown")
public class MM_SkystoneAuto extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    public void runOpMode() {
        robot.init();
        robot.vuforia.init();
        boolean alliance = true;
        String ally = "";
        boolean isHandled = false;
        int skystone = 0;
        while (!opModeIsActive()){ //x is blue which is false, b is red which is true
            if(gamepad1.x) {
                alliance = false;
                ally = "Blue";
            }
            else if (gamepad1.b){
                alliance = true;
                ally = "Red";
            }
            if (gamepad1.y && !isHandled){
                if (skystone < 2){
                    skystone = skystone + 1;
                }else{
                    skystone = 0;
                }
                isHandled = true;
            }
            else if (!gamepad1.y){
                isHandled = false;
            }
            telemetry.addLine("Press button that corresponds to the alliance color");
            telemetry.addLine("Press yellow button for how many skystones");
            telemetry.addData("Current alliance color" , ally);
            telemetry.addData("Number of skystones",skystone);
            telemetry.addLine("Press Play To Start");
            telemetry.update();
        }

        waitForStart();

        robot.drivetrain.resetEncoder();
        robot.drivetrain.driveWithInches(-19,.25);
        int stonePosition = robot.vuforia.getSkystone(alliance);
        robot.drivetrain.driveToSkystone(stonePosition, alliance);
        robot.collector.skystickDown(alliance,stonePosition);
        sleep(1000);
        robot.drivetrain.driveWithInches(12,.5);
        if (alliance){
            robot.drivetrain.gyroTurn(.25,90);
        }
        else{
            robot.drivetrain.gyroTurn(.25,-90);
        }
        if(stonePosition == 1){
            robot.drivetrain.driveWithInches(49.5,.5);
        }
        else{
            robot.drivetrain.driveWithInches(48,.5);
        }
        if (alliance){
            robot.drivetrain.gyroTurn(.25,-90);
        }
        else{
            robot.drivetrain.gyroTurn(.25,90);
        }
        robot.collector.skystickUp(alliance,stonePosition);
        if (skystone > 1) {
//            if (stonePosition == 1) {
//                robot.drivetrain.driveWithInches(60, .5);
//            } else if (stonePosition == 2) {
//                robot.drivetrain.driveWithInches(68, .5);
//            } else {
//                robot.drivetrain.driveWithInches(52, .5);
//            }
            robot.drivetrain.driveWithInches(60, .5);
            robot.drivetrain.driveWithInches(8,.25);
            robot.drivetrain.gyroTurn(.25, 0);
            robot.drivetrain.driveWithInches(-2.5, .25);
            robot.drivetrain.driveToSkystone(stonePosition, alliance);
            robot.collector.skystickDown(alliance,stonePosition);
            sleep(1000);
            robot.drivetrain.driveWithInches(14, .5);
            robot.drivetrain.gyroTurn(.25, 90);
//            if (stonePosition == 1) {
//                robot.drivetrain.driveWithInches(60, .5);
//            } else if (stonePosition == 2) {
//                robot.drivetrain.driveWithInches(68, .5);
//            } else {
//                robot.drivetrain.driveWithInches(52, .5);
//            }
            robot.drivetrain.driveWithInches(68, .5);
            if (alliance){
                robot.drivetrain.gyroTurn(.25,-90);
            }
            else{
                robot.drivetrain.gyroTurn(.25,90);
            }
        }
        robot.collector.skystickUp(alliance,stonePosition);
        robot.drivetrain.driveWithInches(14,.5);
    }
}
