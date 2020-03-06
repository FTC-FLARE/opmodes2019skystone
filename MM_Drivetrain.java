package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import android.view.Display;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.opmodes2019skystone.MM_Robot.toMM;


public class MM_Drivetrain {
    private BNO055IMU imu;
    private DcMotor LMotor = null;
    private DcMotor RMotor = null;
    private Servo foundationServo = null;
    private Servo rightFoundationGrabber = null;
    private Servo leftFoundationGrabber = null;

    private final double P_COEFF = 0.025;  //first attempt .02, PI method .025
    private final double D_COEFF = 0.0025;  //first attempt .0015, PI method .0025
    private final double I_COEFF = 0.00125;  //first attempt .001, PI method .0025
    public static final double HEADING_THRESHOLD = .75;
    private final double RANGE_THRESHOLD = .25;
    private final double MINIMUM_POWER = 0.05;
    private final double WHEEL_DIAMETER = 4.3;
    private final double TICKS_PER_ROTATION = 537.6;
    private final double TICKS_PER_INCH = TICKS_PER_ROTATION / (WHEEL_DIAMETER * 3.14);

    double previousError = 0;
    double derivative = 0;
    double integral = 0;
    double deltaT = 0;

    private LinearOpMode opMode = null;
    private boolean isHandled = false;
    private boolean isFast = true;
    OpenGLMatrix position = null;

    private ModernRoboticsI2cRangeSensor leftRange;
    private ModernRoboticsI2cRangeSensor rightRange;
    private ModernRoboticsI2cRangeSensor backRange;
    private RevTouchSensor leftTouch;
    private RevTouchSensor rightTouch;

    public MM_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        //init motors
        LMotor = opMode.hardwareMap.get(DcMotor.class, "LMotor");
        RMotor = opMode.hardwareMap.get(DcMotor.class, "RMotor");
        LMotor.setDirection(DcMotor.Direction.FORWARD);
        RMotor.setDirection(DcMotor.Direction.REVERSE);
        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //don't forget that the brakes are on
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
        setEncoderTargets(0, 0);

        //init sensors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        leftRange = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"leftRange");
        rightRange = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"rightRange");
        backRange = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backRange");
        leftTouch = opMode.hardwareMap.get(RevTouchSensor.class, "leftTouch");
        rightTouch = opMode.hardwareMap.get(RevTouchSensor.class, "rightTouch");

        //init servo
        foundationServo = opMode.hardwareMap.get(Servo.class, "foundationServo");
        foundationServo.setPosition(0);

        rightFoundationGrabber = opMode.hardwareMap.get(Servo.class, "rightFoundGrab");
        rightFoundationGrabber.setPosition(0);
        leftFoundationGrabber = opMode.hardwareMap.get(Servo.class, "leftFoundGrab");
        leftFoundationGrabber.setPosition(1);
    }

    public void unbrake(){
        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void driveToTouch(double power){
        while(opMode.opModeIsActive() && (!leftTouch.isPressed() || !rightTouch.isPressed())){
            double LPower = power;
            double RPower = power;
            if (leftTouch.isPressed()){
                LPower = 0;
            }
            if(rightTouch.isPressed()){
                RPower = 0;
            }
            setMotorPowers(LPower,RPower);
        }
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
            if (error < 0) power = -power;
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

    //find the power to drive using PID control, don't forget to manipulate the output for the case you are using it in
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



    public void gyroTurn(double speed, double angle){ //180 degrees = .25 speed less than 45 = 1 speed
        runWithEncoder();
        double error = angle - getAngle();
        double power;
        while (opMode.opModeIsActive() && (Math.abs(error) > HEADING_THRESHOLD)) {
            double currentAngle = getAngle();
            error = correctError(angle - currentAngle);
//            power = exponentialControl(error,speed,.0075,2);
            power = exponentialControl(error,speed,.0125,3);
            if (error < 0) {
                power = -power;
            }
            LMotor.setPower(-power);
            RMotor.setPower(power);

            opMode.telemetry.addData("current angle",currentAngle);
            opMode.telemetry.addData("error",error);
            opMode.telemetry.update();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public double exponentialControl(double error, double speed, double coeff, int exponent){
        double power = speed * (Math.pow((Math.abs(error)*coeff),exponent));
        if (Math.abs(power) < MINIMUM_POWER){
            if (power < 0){
                power = -MINIMUM_POWER;
            }else {
                power = MINIMUM_POWER;
            }
        }
        return power;
    }

    public double gyroDriveControl(double error, double speed, double coeff, int exponent){
        return speed * (Math.pow((Math.abs(error)*coeff),exponent));
    }


    public double correctError(double error) {
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

    public void driveToRange(double speed, double inches){
        double error = leftRange.getDistance(DistanceUnit.INCH) - inches;
        double power = 0;
        while (opMode.opModeIsActive() && (Math.abs(error) > RANGE_THRESHOLD)){
            error = backRange.getDistance(DistanceUnit.INCH) - inches;

            if(Math.abs(error) > RANGE_THRESHOLD){
                power = exponentialControl(error, speed, .04, 2);
            }else{
                power = 0;
            }

            if (error < 0) power = -power;

            RMotor.setPower(-power);// the reason the power is negative is because the range sensors are on the back
            LMotor.setPower(-power);
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public double getBackRange(){
        return backRange.getDistance(DistanceUnit.INCH);
    }

    public double getRightRange(){
        return rightRange.getDistance(DistanceUnit.INCH);
    }

    public double getLeftRange(){
        return leftRange.getDistance(DistanceUnit.INCH);
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

    public void driveWithInches(double inches, double speed, MM_Vuforia vuforia) {
        int newLeftTarget;
        int newRightTarget;
        boolean targetSeen = false;

        runWithEncoder();
        runToPosition();

        newLeftTarget = LMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        newRightTarget = RMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        LMotor.setTargetPosition(newLeftTarget);
        RMotor.setTargetPosition(newRightTarget);

        LMotor.setPower(speed);
        RMotor.setPower(speed);

        while (opMode.opModeIsActive() && (RMotor.isBusy() || LMotor.isBusy()) && !targetSeen) {
            position = vuforia.getDistanceToSkyStone();
            if (position != null) {
                targetSeen = true;
            }
            opMode.telemetry.addData("left encoder:", LMotor.getCurrentPosition());
            opMode.telemetry.addData("right encoder:", RMotor.getCurrentPosition());
            opMode.telemetry.addData("target Position:", LMotor.getTargetPosition());
            opMode.telemetry.update();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }


    public void gyroDrive(double inches, double angle, double speed){
        double error = 0;
        double driveError = 0;
        double driveSpeed = speed;
        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorTargets(inches);

        while (opMode.opModeIsActive() && Math.abs(currentPosition()) < Math.abs(inches)){
            error = correctError(angle - getAngle());
            driveError = inches - currentPosition();
            double power = gyroDriveControl(error,speed,.005,1);
            if (driveError < 0) {
                driveSpeed = -speed;
            }
            if (error < 0){
                power = -power;
            }
            LMotor.setPower(driveSpeed-power);
            RMotor.setPower(driveSpeed+power);
            opMode.telemetry.addData("Angle:", getAngle());
            opMode.telemetry.addData("Error:", error);
            opMode.telemetry.addData("drive error",driveError);
            opMode.telemetry.addData("speed", speed);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.addData("left power", LMotor.getPower());
            opMode.telemetry.addData("right power", RMotor.getPower());
            opMode.telemetry.update();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public void setMotorTargets(double inches) {
        int newLeftTarget;
        int newRightTarget;

        runWithEncoder();
        runToPosition();

        newLeftTarget = LMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        newRightTarget = RMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        LMotor.setTargetPosition(newLeftTarget);
        RMotor.setTargetPosition(newRightTarget);
    }


    public void setMotorPowersSame(double power){
        LMotor.setPower(power);
        RMotor.setPower(power);
    }

    public double currentPosition(){
        opMode.telemetry.addData("current position", LMotor.getCurrentPosition());
        opMode.telemetry.addData("current position (calc)",LMotor.getCurrentPosition()/TICKS_PER_INCH);
        return (LMotor.getCurrentPosition()/TICKS_PER_INCH + RMotor.getCurrentPosition()/TICKS_PER_INCH)/2;
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
            rightFoundationGrabber.setPosition(1);
            leftFoundationGrabber.setPosition(0);
        } else {
            foundationServo.setPosition(0);
            rightFoundationGrabber.setPosition(0);
            leftFoundationGrabber.setPosition(1);
        }
    }

    public void grabFoundation(){
        rightFoundationGrabber.setPosition(1);
        leftFoundationGrabber.setPosition(0);
    }

    public void releaseFoundation(){
        rightFoundationGrabber.setPosition(0);
        leftFoundationGrabber.setPosition(1);
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

    public void runWithEncoder() {
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