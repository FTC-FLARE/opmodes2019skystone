package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MM_Drivetrain {
    private LinearOpMode opMode = null;

    private DcMotor LMotor = null;
    private DcMotor RMotor = null;

    private final double wheelDiameter = 4.25;
    private final double ticksPerRotation = 723.24;
    private final double ticksPerInch = (4.25*3.14)*723.24;

    public MM_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        LMotor = opMode.hardwareMap.get(DcMotor.class, "LMotor");
        RMotor = opMode.hardwareMap.get(DcMotor.class, "RMotor");

        LMotor.setDirection(DcMotor.Direction.REVERSE);
        RMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void driveWithSticks(double leftPower, double rightPower) {
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);
    }
}