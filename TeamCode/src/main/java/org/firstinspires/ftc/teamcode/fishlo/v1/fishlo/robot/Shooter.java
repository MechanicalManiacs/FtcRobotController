package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition.DropNShootRoadRunnerAuto;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Shooter extends SubSystem {

    private Servo pusher;
    private DcMotor shooter;

    public static final double PUSHER_HOME = 1;
    public static final double PUSHER_MAX = 0.85;
    public static double RAMP_ANGLE = 30;
    double shooter_power;
    boolean shooter_started = false;
    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);


    public Shooter(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        shooter = robot.hardwareMap.dcMotor.get("shooter");
        pusher = robot.hardwareMap.servo.get("pusher");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        mecanumDrive.setPoseEstimate(DropNShootRoadRunnerAuto.endPose);
    }

    @Override
    public void handle() {
        mecanumDrive.update();
        Pose2d drivePose = mecanumDrive.getPoseEstimate();
        Pose2d goalPose = new Pose2d(64, -36);

        double goalDistance = Math.sqrt(Math.pow(drivePose.getX() - goalPose.getX(), 2) +
                Math.pow(drivePose.getY() - goalPose.getY(), 2));

        double goalAngle = Math.tan((drivePose.getY()-goalPose.getY())/(drivePose.getX()-goalPose.getX()));

        robot.telemetry.addData("Goal Distance: ", goalDistance);
        robot.telemetry.update();

        double shooter_speed = Math.sqrt((goalDistance * 9.8)/Math.sin(2 * RAMP_ANGLE));
        shooter_power = 7.2 / shooter_speed;
        if (robot.gamepad2.right_bumper){
            mecanumDrive.turn(Math.toRadians(90-goalAngle));
            startShooter();
        }
        else if (robot.gamepad2.left_bumper) {
            stopShooter();
        }
        if (robot.gamepad2.a && shooter_started) {
            shoot();
        }
        if (robot.gamepad2.dpad_down) {
            resetPusher();
        }
    }

    @Override
    public void stop() {
        stopShooter();
        resetPusher();
    }

    public void shoot() {
        pusher.setPosition(PUSHER_MAX);
    }

    public void startShooter() {
        shooter.setPower(shooter_power);
        shooter_started = true;
    }

    public void stopShooter() {
        shooter.setPower(0);
        shooter_started = false;
    }

    public void startShooterAuto(double distance) {
        double shooter_speed = Math.sqrt((distance * 9.8)/Math.sin(2 * RAMP_ANGLE));
        shooter_power = 7.2 / shooter_speed;
        shooter.setPower(shooter_power);
        shooter_started = true;
    }




    public void resetPusher() {
        pusher.setPosition(PUSHER_HOME);
    }
}
