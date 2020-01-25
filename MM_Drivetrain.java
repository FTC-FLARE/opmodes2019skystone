package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class MM_Drivetrain {
    private BNO055IMU imu;
    private DcMotor LMotor = null;
    private DcMotor RMotor = null;
    private Servo foundationServo = null;

    private final double P_COEFF = 0.025;  //first attempt .02, PI method .025
    private final double D_COEFF = 0.0025;  //first attempt .0015, PI method .0025
    private final double I_COEFF = 0.00125;  //first attempt .001, PI method .0025
    private final double HEADING_THRESHOLD = 1;
    private final double MINIMUM_POWER = .075;
    private final double WHEEL_DIAMETER = 4.3;
    private final double TICKS_PER_ROTATION = 723.24;
    private final double TICKS_PER_INCH = TICKS_PER_ROTATION / (WHEEL_DIAMETER * 3.14);

    double previousError = 0;
    double derivative;
    double integral = 0;
    double deltaT = 0;

    private LinearOpMode opMode = null;
    private boolean isHandled = false;
    private boolean isFast = true;

    private DistanceSensor leftRange;
    private DistanceSensor rightRange;

    public MM_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        //init motors
        LMotor = opMode.hardwareMap.get(DcMotor.class, "LMotor");
        RMotor = opMode.hardwareMap.get(DcMotor.class, "RMotor");
        LMotor.setDirection(DcMotor.Direction.REVERSE);
        RMotor.setDirection(DcMotor.Direction.FORWARD);
        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //don't forget that the brakes are on
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
        setEncoderTargets(0, 0);

        //init gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //init servo
        foundationServo = opMode.hardwareMap.get(Servo.class, "foundationServo");
        foundationServo.setPosition(0);
    }

    public void gyroTurnPID(double speed, double angle) {
        double power;
        double error = angle - getAngle();
        opMode.resetStartTime();
        while (opMode.opModeIsActive() && (Math.abs(error) > HEADING_THRESHOLD)){
            error = correctError(error);
            power = PIDcontrol(speed,error);
            if((power < MINIMUM_POWER && power > 0) || (power > -MINIMUM_POWER && power < 0)) {
                if (power > 0){
                    power = MINIMUM_POWER;
                }else{
                    power = -MINIMUM_POWER;
                }
            }
            if (error < 0) {
                power = -power;
            }
            LMotor.setPower(-power);
            RMotor.setPower(power);
            opMode.telemetry.addData("Actual Angle", getAngle());
            opMode.telemetry.addData("Target Angle",angle);
            opMode.telemetry.addData("Error",error);
            opMode.telemetry.addData("Left Power",LMotor.getPower());
            opMode.telemetry.addData("Right Power",RMotor.getPower());
            opMode.telemetry.update();
            error = angle - getAngle();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public double PIDcontrol(double speed, double error){
        double power;

        deltaT = opMode.getRuntime();
        integral = integral + (error * deltaT);
        derivative = (error - previousError) / deltaT;
        opMode.resetStartTime();
        power = speed * (Math.abs(error) * P_COEFF) + (derivative * D_COEFF) + (integral * I_COEFF);
        previousError = error;
        return power;
    }



    public void gyroTurn(double speed, double angle){
        double error = angle - getAngle();
        double power;
        while (opMode.opModeIsActive() && (Math.abs(error) > HEADING_THRESHOLD)) {
            error = correctError(angle - getAngle());
            power = speed * (Math.pow((Math.abs(error) * .0125), 2) + MINIMUM_POWER);
            if (error < 0) {
                power = -power;
            }
            LMotor.setPower(-power);
            RMotor.setPower(power);

        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    private double correctError(double error) {
        if(error > 180){
            error = error - 360;
        }else if(error < -180) {
            error = error + 360;
        }
        return error;
    }

    public double getAngle(){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        return angle;
    }

    public void driveWithInches(double inches, double speed) {
        int newLeftTarget;
        int newRightTarget;

        runWithEncoder();
        runToPosition();

        newLeftTarget = LMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        newRightTarget = RMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        LMotor.setTargetPosition(newLeftTarget);
        RMotor.setTargetPosition(newRightTarget);

        LMotor.setPower(speed);
        RMotor.setPower(speed);

        while (opMode.opModeIsActive() && (RMotor.isBusy() || LMotor.isBusy())) {
            opMode.telemetry.addData("left encoder:", LMotor.getCurrentPosition());
            opMode.telemetry.addData("right encoder:", RMotor.getCurrentPosition());
            opMode.telemetry.addData("target Position:", LMotor.getTargetPosition());
            opMode.telemetry.update();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public void setMotorPowersSame(double power){
        LMotor.setPower(power);
        RMotor.setPower(power);
    }

    public double currentPosition(){
        opMode.telemetry.addData("current position", LMotor.getCurrentPosition());
        opMode.telemetry.addData("current position (calc)",LMotor.getCurrentPosition()/TICKS_PER_INCH);
        return LMotor.getCurrentPosition()/TICKS_PER_INCH;
    }

    public void driveWithSticks() {
        double leftPower = -opMode.gamepad1.left_stick_y;
        double rightPower = -opMode.gamepad1.right_stick_y;
        if (opMode.gamepad1.a && !isHandled) {
            isFast = !isFast;
            isHandled = true;
        } else if (!opMode.gamepad1.a && isHandled) {
            isHandled = false;
        }
        if(!isFast){
            leftPower = leftPower / 4;
            rightPower = rightPower / 4;
        }

        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);
    }

    public void controlFoundation(){
        if (opMode.gamepad2.a) {
            foundationServo.setPosition(1);
        } else {
            foundationServo.setPosition(0);
        }
    }

    public void runToPosition() {
        LMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder() {
        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runWithEncoder();
    }

    private void runWithEncoder() {
        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder() {
        LMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setEncoderTargets(int left, int right){
        LMotor.setTargetPosition(left);
        RMotor.setTargetPosition(right);
    }

    public void foundationUp() {
        foundationServo.setPosition(0);
    }

    public void foundationDown() {
        foundationServo.setPosition(1);
    }

    //robot is backwards so all distances are negative
    //blue alliance is false, red is true (same for all uses)
    public void driveLeftConfig(boolean alliance) {
        if (alliance){
            driveWithInches(-8.5,.25);
        }
        else {
            driveWithInches(-8.5,.25);
        }
    }

    public void driveCenterConfig(boolean alliance) {
        if (alliance) {
            gyroTurn(.25,-20);
            driveWithInches(-8.5,.25);
        }
        else {
            gyroTurn(.25,20);
            driveWithInches(-8.5,.25);
        }
    }

    public void driveRightConfig(boolean alliance) {
        if (alliance) {
            driveWithInches(-8.5,.25);
        }
        else {
            driveWithInches(-8.5,.25);
        }
    }

    public void driveToSkystone(int stonePosition, boolean alliance) {
        if (stonePosition == 0) {
            driveLeftConfig(alliance);
        }
        else if(stonePosition == 1) {
            driveCenterConfig(alliance);
        }
        else{
            driveRightConfig(alliance);
        }
    }

    public void setMotorPowers(double leftPower, double rightPower){
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);
    }
}