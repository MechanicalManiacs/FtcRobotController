package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition.DropNShootRoadRunnerAuto;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import java.util.Vector;

public class Drive extends SubSystem {

    SampleMecanumDriveCancelable mecanumDrive;

    Gyro gyro = new Gyro(robot);

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    int cpr = 28;
    int gearRatio = 19;
    double diameter = 3.780;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.955;
    double strafeBias = 0.9;
    double conversion = cpi * bias;
    Claw claw = new Claw(robot);


    private enum DriveControls {
        TANK,
        FIELD,
        ARCADE
    }
    private enum mode {
        NORMAL_CONTROL,
        GO_TO_POINT
    }

    private mode currentMode = mode.NORMAL_CONTROL;

    Vector2d shootingPose = new Vector2d(0, -40);
    double shootingHeading = Math.toRadians(0);

    DriveControls[] driveControls = {DriveControls.ARCADE, DriveControls.FIELD, DriveControls.TANK};
    DriveControls driveType;
    int driveIndex = 2;

    boolean exit = false;

    public Drive(Robot robot) {

        super(robot);
    }

    @Override
    public void init() {
        mecanumDrive = new SampleMecanumDriveCancelable(robot.hardwareMap);

        frontLeft = null;
        backLeft = null;
        frontLeft = robot.hardwareMap.dcMotor.get("frontLeft");
        frontRight = robot.hardwareMap.dcMotor.get("frontRight");
        backLeft = robot.hardwareMap.dcMotor.get("backLeft");
        backRight = robot.hardwareMap.dcMotor.get("backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        drive(0, 0);

        if (DropNShootRoadRunnerAuto.autoEnded) {
            mecanumDrive.setPoseEstimate(DropNShootRoadRunnerAuto.endPose);
        }
        else {
            mecanumDrive.setPoseEstimate(new Pose2d(-63, -49, 180));
        }

        gyro.initGyro();

        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mecanumDrive.setPoseEstimate(PoseStorage.currentPose);
    }

    boolean reverse = false;

    @Override
    public void handle() {

        double driveSpeed = -robot.gamepad1.left_stick_y;
        double rightY = robot.gamepad1.right_stick_y;
        double turnSpeed = 0;
        double strafeSpeed = robot.gamepad1.left_stick_x;


        if (Math.abs(robot.gamepad1.right_stick_x) > 0.1) {
            turnSpeed = robot.gamepad1.right_stick_x;
        }

        if(robot.gamepad1.x) {
            driveIndex = 0;
        }
        if(robot.gamepad1.b) {
            driveIndex = 2;
        }

        mecanumDrive.update();

        driveType = driveControls[driveIndex];

        switch (currentMode) {
            case NORMAL_CONTROL:
                runDrive(driveType, driveSpeed, strafeSpeed, turnSpeed, rightY, -driveSpeed);
                if (robot.gamepad1.a) {
                    moveToShootingPosition();
                    currentMode = mode.GO_TO_POINT;
                }
                break;
            case GO_TO_POINT:
                if (robot.gamepad1.y) {
                    mecanumDrive.cancelFollowing();
                    currentMode = mode.NORMAL_CONTROL;
                }
                if (!mecanumDrive.isBusy()) {
                    currentMode = mode.NORMAL_CONTROL;
                }
        }


        robot.telemetry.addData("Drive - Dat - Drive Controls", driveType.name());
        robot.telemetry.addData("Drive - Dat - Drive Speed", driveSpeed);
        robot.telemetry.addData("Drive - Dat - Turn Speed", turnSpeed);
        robot.telemetry.addData("Drive - Dat - Strafe Speed", strafeSpeed);
        robot.telemetry.addData("mode", currentMode);


    }

    public void runDrive(DriveControls driveType, double driveSpeed, double strafeSpeed, double turnSpeed, double rightY, double leftY) {
        if (driveType == DriveControls.ARCADE) {
            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            driveSpeed,
                            -strafeSpeed,
                            -turnSpeed
                    )
            );
        }

        if (driveType == DriveControls.FIELD) {
            // Read pose
            Pose2d poseEstimate = mecanumDrive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    driveSpeed,
                    -strafeSpeed
            ).rotated(-poseEstimate.getHeading() + 90); //Change + to - if it doesnt work properly

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -turnSpeed
                    )
            );
        }
        if (driveType == DriveControls.TANK) {

            if (robot.gamepad1.right_bumper) {
                if (robot.gamepad1.right_trigger < 0.5) {
                    mecanumDrive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    -0.5,
                                    0
                            )
                    );
                }
                else if (robot.gamepad1.left_trigger < 0.5) {
                    mecanumDrive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    -0.3,
                                    0
                            )
                    );
                }
                else {
                    mecanumDrive.setWeightedDrivePower(
                             new Pose2d(
                                     0,
                                     -1,
                                     0
                             )
                    );
                }
            }
            else if (robot.gamepad1.left_bumper) {
                if (robot.gamepad1.right_trigger < 0.5) {
                    mecanumDrive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    0.5,
                                    0
                            )
                    );
                }
                else if (robot.gamepad1.left_trigger < 0.5) {
                    mecanumDrive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    0.3,
                                    0
                            )
                    );
                }
                else {
                    mecanumDrive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    1,
                                    0
                            )
                    );
                }
            }
            else {
                left(-robot.gamepad1.left_stick_y);
                right(-robot.gamepad1.right_stick_y);
            }


        }
    }

    private void moveToShootingPosition() {
        Trajectory trajectory = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                .splineTo(new Vector2d(0, -40), Math.toRadians(0))
                .build();
        mecanumDrive.followTrajectoryAsync(trajectory);
    }

    private void moveToPowershot() {
        Trajectory trajectory = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                .splineTo(new Vector2d(-63, 0), Math.toRadians(90))
                .build();
        mecanumDrive.followTrajectoryAsync(trajectory);
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

    public void turnWithEncoder(int move, double speed) {

        encoderReset();

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
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

    /**
     * tests
     */

    public void testFrontLeft() {
        frontLeft.setPower(0.2);
    }
    public void testFrontRight() {
        frontRight.setPower(0.2);
    }
    public void testBackLeft() {
        backLeft.setPower(0.2);
    }
    public void testBackRight() {
        backRight.setPower(0.2);
    }





}
