package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_Robot {
    private LinearOpMode opMode;
    public MM_Drivetrain drivetrain;

    public MM_Robot(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
    }

    public void driveWithSticks() {
        drivetrain.driveWithSticks();
    }
}
