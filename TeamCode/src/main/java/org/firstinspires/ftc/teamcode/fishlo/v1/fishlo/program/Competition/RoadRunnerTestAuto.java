package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class RoadRunnerTestAuto extends FishloAutonomousProgram {

    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);

    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void preMain() {
        telemetry.addLine("Ready");
        Pose2d startPose = new Pose2d(-54, -54);
        mecanumDrive.setPoseEstimate(startPose);
    }

    @Override
    public void main() {
        Trajectory traj1 = mecanumDrive.trajectoryBuilder(new Pose2d())
            .lineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(90)))
            .build();
        mecanumDrive.followTrajectory(traj1);

        Trajectory traj2 = mecanumDrive.trajectoryBuilder(traj1.end())
            .forward(100)
            .build();
        mecanumDrive.followTrajectory(traj2);
    }
}
