package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Gyro;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive extends SubSystem {

    Gyro gyro = new Gyro(robot);
    ElapsedTime teleop_timer = new ElapsedTime();

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    int cpr = 28;
    int gearRatio = 19;
    double diameter = 3.780;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.955;
    double strafeBias = 0.9;
    double conversion = cpi * bias;

    boolean exit = false;
    
    public Drive(Robot robot) {

        super(robot);
    }

    @Override
    public void init() {
        frontLeft = robot.hardwareMap.dcMotor.get("frontLeft");
        frontRight = robot.hardwareMap.dcMotor.get("frontRight");
        backLeft = robot.hardwareMap.dcMotor.get("backLeft");
        backRight = robot.hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        drive(0, 0);

        gyro.initGyro();
    }

    boolean reverse = false;
    boolean FieldOriented = false;

    @Override
    public void handle() {
        double driveSpeed = -robot.gamepad1.left_stick_y;
        double turnSpeed = 0;
        double strafeSpeed = robot.gamepad1.left_stick_x;

        if (Math.abs(robot.gamepad1.right_stick_x) > 0.1) {
            turnSpeed = robot.gamepad1.right_stick_x;
        }

        if(robot.gamepad1.a) {
            reverse = false;
        }
        else if(robot.gamepad1.y) {
            reverse = true;
        }

        drive(driveSpeed, driveSpeed);
        strafe(strafeSpeed);
        drive(turnSpeed, -turnSpeed);
//        if (robot.gamepad1.dpad_up && !FieldOriented) {
//            runDrive("arcade", driveSpeed, strafeSpeed, turnSpeed);
//        }
//        if (robot.gamepad1.dpad_down && FieldOriented) {
//            gyro.resetHeading();
//            runDrive("field", driveSpeed, strafeSpeed, turnSpeed);
//        }


        robot.telemetry.addData("Drive - Dat - Drive Speed", driveSpeed);
        robot.telemetry.addData("Drive - Dat - Turn Speed", turnSpeed);
        robot.telemetry.addData("Drive - Dat - GamepadX", robot.gamepad1.left_stick_x);
        robot.telemetry.addData("Drive - Dat - Strafe Speed", strafeSpeed);
        robot.telemetry.addData("Drive - Set - frontLeft", frontLeft.getPower());
        robot.telemetry.addData("Drive - Set - backLeft", backLeft.getPower());
        robot.telemetry.addData("Drive - Set - frontRight", frontRight.getPower());
        robot.telemetry.addData("Drive - Set - backRight", backRight.getPower());
        robot.telemetry.addData("Drive - Enc - Left", frontLeft.getCurrentPosition());
        robot.telemetry.addData("Drive - Enc - Right", frontRight.getCurrentPosition());
        robot.telemetry.update();
    }

    public void runDrive(String drive, double driveSpeed, double strafeSpeed, double turnSpeed) {
        if (drive.equalsIgnoreCase("arcade")) {
            drive(driveSpeed, driveSpeed);
            strafe(strafeSpeed);
            drive(turnSpeed, -turnSpeed);
        }
        else if (drive.equalsIgnoreCase("field")) {
            double gyroDegrees = gyro.getHeading() + 90;
            double gyroRadians = gyroDegrees * Math.PI/180;
            double temp = driveSpeed * Math.cos(gyroRadians) + strafeSpeed * Math.sin(gyroRadians);
            strafeSpeed = -driveSpeed * Math.sin(gyroRadians) + strafeSpeed * Math.cos(gyroRadians);
            driveSpeed = temp;

            drive(driveSpeed, driveSpeed);
            strafe(strafeSpeed);
            drive(turnSpeed, -turnSpeed);
        }
        else {
            robot.telemetry.addData("Error", "Incorrect input");
        }
    }

    private void left(double power) {
        try {
            frontLeft.setPower(power);
            backLeft.setPower(power);
        } catch(Exception ex) {}
    }

    private void right(double power) {
        try {
            frontRight.setPower(power);
            backRight.setPower(power);
        } catch(Exception ex) {}
    }

    public void drive(double leftPower, double rightPower) {
        left(leftPower);
        right(rightPower);
    }

    public void strafe(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    public void turn (double power) {

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    public void moveToPosition(double inches, double speed) {
        int move = (int)(Math.round(inches*conversion));

        encoderReset();

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(speed, speed);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            if (exit) {
                return;
            }
        }
        stop();
        encoderReset();
        return;
    }


    public void strafeToPosition(double inches, double speed) {
        int move = (int)(Math.round(inches * cpi * strafeBias));

        encoderReset();

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(speed, speed);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {}
        stop();
        encoderReset();
        return;
    }

    public void stop() {
        drive(0, 0);
    }

/*
    int base = 0;
    public void resetEncoder() {
        base = backRight.getCurrentPosition();
    }
*/

    public void encoderReset() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

/*
    public int getEncoder() {
        return backRight.getCurrentPosition() - base;
    }
*/



}
