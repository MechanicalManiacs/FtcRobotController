package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Disabled
@Autonomous
public class PositionTester extends FishloAutonomousProgram {
    SampleMecanumDrive mecanumDrive;
    Trajectory moveToPowerShot1;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-63, -49, Math.toRadians(180));;
        moveToPowerShot1 = mecanumDrive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(72, 45), Math.toRadians(180))
                .build();
        telemetry.addLine("Ready");
    }

    @Override
    public void main() {
        mecanumDrive.followTrajectory(moveToPowerShot1);
    }

}
