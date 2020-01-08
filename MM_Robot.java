package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import static android.os.SystemClock.sleep;

public class MM_Robot {
    private LinearOpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Arm arm;
    public MM_Vuforia vuforia;

    OpenGLMatrix position = null;

    public MM_Robot(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        arm = new MM_Arm(opMode);
        vuforia = new MM_Vuforia(opMode);
    }
    public void driveToTarget(){
        boolean targetSeen = false;
        drivetrain.setMotorPowersSame(-.125);
        while(opMode.opModeIsActive() && !targetSeen && drivetrain.currentPosition() > -16){
            position = vuforia.getDistanceToSkyStone();
            if(position != null){
                targetSeen = true;
            }
        }
        drivetrain.setMotorPowersSame(0);
        if (!targetSeen) {
            drivetrain.gyroTurn(.25,45);
            while(opMode.opModeIsActive() && !targetSeen){
                position = vuforia.getDistanceToSkyStone();
                if(position != null){
                    targetSeen = true;
                }
                sleep(5);
            }
        }
    }
    public void driveToSkystone(float xDistance, float yDistance){

    }
}
