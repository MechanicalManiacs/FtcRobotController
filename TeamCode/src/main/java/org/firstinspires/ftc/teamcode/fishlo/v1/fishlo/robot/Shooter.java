package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition.DropNShootRoadRunnerAuto;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import java.util.HashMap;

public class Shooter extends SubSystem {

    private Servo pusher;
    private DcMotor shooter;

    public static final double PUSHER_HOME = 1;
    public static final double PUSHER_MAX = 0.85;
    public static double RAMP_ANGLE = 30;
    double shooter_power;
    boolean shooter_started = false;
    private SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);
    private enum Goals {
        LOW,
        MIDDLE,
        HIGH,
        POWER_SHOT_1,
        POWER_SHOT_2,
        POWER_SHOT_3
    }
    private HashMap<Goals, Pose3d> goalMap = new HashMap<Goals, Pose3d>();

    private Goals[] targets = {Goals.LOW, Goals.MIDDLE, Goals.HIGH, Goals.POWER_SHOT_1, Goals.POWER_SHOT_2, Goals.POWER_SHOT_3};
    private Goals target;
    int targetIndex = 2;
    public Shooter(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        shooter = robot.hardwareMap.dcMotor.get("shooter");
        pusher = robot.hardwareMap.servo.get("pusher");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set target positions in hash map
        goalMap.put(Goals.LOW, new Pose3d(new Pose2d(74, -36, 180), 17));
        goalMap.put(Goals.MIDDLE, new Pose3d(new Pose2d(74, -36, 180), 25));
        goalMap.put(Goals.HIGH, new Pose3d(new Pose2d(74, -36, 180), 35));
        goalMap.put(Goals.POWER_SHOT_1, new Pose3d(new Pose2d(74.5, 45, 180), 30));
        goalMap.put(Goals.POWER_SHOT_2, new Pose3d(new Pose2d(74.5, 37.5, 180), 30));
        goalMap.put(Goals.POWER_SHOT_3, new Pose3d(new Pose2d(74.5, 30, 180), 30));

        mecanumDrive.setPoseEstimate(DropNShootRoadRunnerAuto.endPose);
    }

    @Override
    public void handle() {

        if (robot.gamepad2.dpad_right) {
            targetIndex++;
            if (targetIndex > 5) {
                targetIndex = 0;
            }
        }
        if (robot.gamepad2.dpad_left) {
            targetIndex--;
            if (targetIndex < 0) {
                targetIndex = 5;
            }
        }
        target = targets[targetIndex];

        mecanumDrive.update();
        Pose2d drivePose = mecanumDrive.getPoseEstimate();
        Pose2d goalPose = goalMap.get(target).getPosition();
        double height = goalMap.get(target).getHeight();

        double goalDistance = Math.sqrt(Math.pow(drivePose.getX() - goalPose.getX(), 2) +
                Math.pow(drivePose.getY() - goalPose.getY(), 2));
        double goalAngle = Math.atan(drivePose.getX()-goalPose.getX())/(drivePose.getY()-goalPose.getY());
        double GRAVITY = 9.8;

        double shooter_speed = Math.sqrt(
                (GRAVITY * Math.pow(goalDistance, 2)) /
                ( 2 * Math.pow(Math.cos(RAMP_ANGLE), 2) * (Math.tan(RAMP_ANGLE) * goalDistance - height))
                );


        robot.telemetry.addData("Goal: ", target.name());
        robot.telemetry.addData("Goal Distance: ", goalDistance);
        robot.telemetry.addData("Goal Angle: ", goalAngle);
        robot.telemetry.update();

        shooter_speed = Math.max(0, Math.min(7.2, shooter_speed));
        shooter_power = 7.2/shooter_speed;

        if (robot.gamepad2.right_bumper){
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

class Pose3d {
    private Pose2d position;
    private double height;

    public Pose3d(Pose2d pos, double h) {
        position = pos;
        height = h;
    }

    public Pose2d getPosition() {
        return position;
    }

    public double getHeight() {
        return height;
    }
}