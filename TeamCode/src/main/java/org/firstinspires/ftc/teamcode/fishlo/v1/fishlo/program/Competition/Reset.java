package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class Reset extends FishloAutonomousProgram {
    SampleMecanumDrive mecanumDrive;
    Trajectory returnTrajectory;
    Trajectory forwardTrajectory;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        mecanumDrive.setPoseEstimate(DropNShootRoadRunnerAuto.endPose);

        forwardTrajectory = mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate())
                .splineTo(new Vector2d(-5, -24), Math.toRadians(270))
                .build();

        returnTrajectory =  mecanumDrive.trajectoryBuilder(forwardTrajectory.end())
                .splineTo(new Vector2d(-67, -57), Math.toRadians(180))
                .build();

        telemetry.addLine("Ready");
        telemetry.update();

    }

    @Override
    public void main() {
        mecanumDrive.followTrajectory(forwardTrajectory);
        mecanumDrive.followTrajectory(returnTrajectory);
    }


}
