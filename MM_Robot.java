package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_Robot {
    private LinearOpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Arm arm;
    public MM_Vuforia vuforia;

    public MM_Robot(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        arm = new MM_Arm(opMode);
        vuforia = new MM_Vuforia(opMode);
    }
    public void driveToSkystone (){
        int stonePosition = vuforia.getSkystone();
        drivetrain.distanceToSkystone(stonePosition);
    }
}
