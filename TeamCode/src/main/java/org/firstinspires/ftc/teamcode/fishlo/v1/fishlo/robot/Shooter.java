package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition.DropNShootRoadRunnerAuto;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import java.util.HashMap;

public class Shooter extends SubSystem {

    private CRServo pusher;
    private DcMotor shooter;

    public static double RAMP_ANGLE = 30;
    double shooter_power;
    public boolean shooter_started = false;
    private SampleMecanumDrive mecanumDrive;

    ElapsedTime timer = new ElapsedTime();

    public enum Goals {
        LOW,
        MIDDLE,
        HIGH,
        POWER_SHOT_1,
        POWER_SHOT_2,
        POWER_SHOT_3
    }

    private enum Modes {
        AUTOMATIC,
        OVERRIDE
    }

    public HashMap<Goals, Pose3d> goalMap = new HashMap<Goals, Pose3d>();

    private Goals[] targets = {Goals.LOW, Goals.MIDDLE, Goals.HIGH, Goals.POWER_SHOT_1, Goals.POWER_SHOT_2, Goals.POWER_SHOT_3};
    private Goals target;
    private Modes mode = Modes.AUTOMATIC;
    int targetIndex = 2;
    public Shooter(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        shooter = robot.hardwareMap.dcMotor.get("shooter");
        pusher = robot.hardwareMap.crservo.get("pusher");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);

        // Set target positions in hash map
        goalMap.put(Goals.LOW, new Pose3d(new Pose2d(74, -36, 180), 17));
        goalMap.put(Goals.MIDDLE, new Pose3d(new Pose2d(74, -36, 180), 25));
        goalMap.put(Goals.HIGH, new Pose3d(new Pose2d(74, -36, 180), 35));
        goalMap.put(Goals.POWER_SHOT_1, new Pose3d(new Pose2d(74.5, 45, 180), 30));
        goalMap.put(Goals.POWER_SHOT_2, new Pose3d(new Pose2d(74.5, 37.5, 180), 30));
        goalMap.put(Goals.POWER_SHOT_3, new Pose3d(new Pose2d(74.5, 30, 180), 30));

        if (DropNShootRoadRunnerAuto.autoEnded) {
            mecanumDrive.setPoseEstimate(DropNShootRoadRunnerAuto.endPose);
        }
        else {
            mecanumDrive.setPoseEstimate(new Pose2d(-63, -49, 180));
        }
    }

    @Override
    public void handle() {

        //Driver can cycle through the targets using the d-pad
        if (robot.gamepad2.dpad_up) {
            targetIndex++;
            if (targetIndex > 5) {
                targetIndex = 0;
            }
        }
        if (robot.gamepad2.dpad_down) {
            targetIndex--;
            if (targetIndex < 0) {
                targetIndex = 5;
            }
        }
        if (robot.gamepad2.dpad_right) {
            targetIndex = 2;
        }
        if (robot.gamepad2.dpad_left) {
            targetIndex = 4;
        }
        target = targets[targetIndex];

        mecanumDrive.update();
        Pose2d drivePose = mecanumDrive.getPoseEstimate();
        Pose2d goalPose = goalMap.get(target).getPosition();
        double height = goalMap.get(target).getHeight() * 0.0254;

        double goalDistance = Math.sqrt(Math.pow(drivePose.getX() - goalPose.getX(), 2) +
                Math.pow(drivePose.getY() - goalPose.getY(), 2)) * 0.0254;
        double goalAngle = Math.toDegrees(Math.atan((drivePose.getX()-goalPose.getX())
                /(drivePose.getY()-goalPose.getY())));
        double GRAVITY = 9.8;

        double shooter_speed = Math.sqrt(
                (GRAVITY * Math.pow(goalDistance, 2)) /
                        ( 2 * Math.pow(Math.cos(Math.toRadians(RAMP_ANGLE)), 2) * (Math.tan(Math.toRadians(RAMP_ANGLE)) * goalDistance - height))
                );


        //Driver can manually override the shooter power
        if (robot.gamepad2.left_stick_button) {
            mode = Modes.OVERRIDE;
        }
        if (robot.gamepad2.right_stick_button){
            mode = Modes.AUTOMATIC;
        }
        if (mode == Modes.AUTOMATIC) {
            shooter_power = Math.max(0, Math.min(1, shooter_power));
        }
        if (mode == Modes.OVERRIDE) {
            shooter_power = 1;
        }

        if (robot.gamepad2.right_bumper){
            startShooter();
        }
        else if (robot.gamepad2.left_bumper) {
            stopShooter();
        }
        if (robot.gamepad2.a && shooter_started) {
            shoot();
        }

        robot.telemetry.addData("Mode: ", mode);
        robot.telemetry.addData("Goal: ", target.name());
        robot.telemetry.addData("Goal Distance: ", goalDistance);
        robot.telemetry.addData("Goal Angle: ", goalAngle);
        robot.telemetry.addData("Shooting Speed (Max is 7.2): ", shooter_speed);
        robot.telemetry.addData("Shooting Power", shooter_power);
    }

    @Override
    public void stop() {
        stopShooter();
    }

    public void shoot() {
        timer.reset();
        while (timer.milliseconds() < 400) {
            pusher.setPower(0.3);
        }
        pusher.setPower(0);
    }

    public void startShooter() {
        shooter.setPower(shooter_power);
        shooter_started = true;
    }


    public void stopShooter() {
        shooter.setPower(0);
        shooter_started = false;
    }

    public void startShooterAuto(Goals goal, Pose2d drivePose) {
        Pose2d goalPose = goalMap.get(goal).getPosition();
        double height = goalMap.get(goal).getHeight();

        double goalDistance = Math.sqrt(Math.pow(drivePose.getX() - goalPose.getX(), 2) +
                Math.pow(drivePose.getY() - goalPose.getY(), 2));
        double goalAngle = Math.atan((drivePose.getX()-goalPose.getX())
                /(drivePose.getY()-goalPose.getY()));
        double GRAVITY = 9.8;


        double shooter_speed = Math.sqrt(
                (GRAVITY * Math.pow(goalDistance, 2)) /
                        ( 2 * Math.pow(Math.cos(Math.toRadians(RAMP_ANGLE)), 2) * (Math.tan(Math.toRadians(RAMP_ANGLE)) * goalDistance - height))
        );

        shooter_speed = Math.max(0, Math.min(7.2, shooter_speed));
        robot.telemetry.addData("Shooter Speed", shooter_speed);
        robot.telemetry.addData("Shooting Angle", Math.toDegrees(goalAngle));

        shooter_power = 7.2 / shooter_speed;

        shooter.setPower(shooter_power);
        mecanumDrive.turn(goalAngle);
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