package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.OpenCV;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class DropNShootRoadRunnerAuto extends FishloAutonomousProgram {

    //Back right corner is (-72, -72)
    //Back left corner is (-72, 24)
    //Front right corner is (72, -72)
    //Front left corner is (72, 24)

    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPose;
    protected OpenCV.targetZone targetZone;

    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void preMain() {
        telemetry.addLine("Ready");
        startPose = new Pose2d(-54 -54);
        mecanumDrive.setPoseEstimate(startPose);
        openCV.initVision();

        while (!isStarted()) {

            targetZone = openCV.getTargetZone();
            telemetry.addData("TargetZone", targetZone);
            telemetry.addData("Rings", openCV.getHeight());

        }
    }

    @Override
    public void main() {
        openCV.stopAll();

        /**
         * Initialize Trajectories
         */

        //First Trajectory - Move to Target Zone
        Trajectory targetZoneATraj1 = mecanumDrive.trajectoryBuilder(startPose)
            .splineTo(new Vector2d(6, -54), Math.toRadians(0))
            .splineTo(new Vector2d(30, -42), Math.toRadians(0))
            .build();

        Trajectory targetZoneBTraj1 = mecanumDrive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(6, -54), Math.toRadians(0))
                .splineTo(new Vector2d(54, -18), Math.toRadians(0))
                .build();

        Trajectory targetZoneCTraj1 = mecanumDrive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(6, -54), Math.toRadians(0))
                .splineTo(new Vector2d(76, -42), Math.toRadians(0))
                .build();



        //Second Trajectory - Move to second wobble goal
        Trajectory targetZoneATraj2 = mecanumDrive.trajectoryBuilder(targetZoneATraj1.end(), true)
                .splineTo(new Vector2d(6, -34), Math.toRadians(0))
                .splineTo(new Vector2d(-6, -22), Math.toRadians(0))
                .build();

        Trajectory targetZoneBTraj2 = mecanumDrive.trajectoryBuilder(targetZoneBTraj1.end(), true)
                .splineTo(new Vector2d(6, -34), Math.toRadians(0))
                .splineTo(new Vector2d(-6, -22), Math.toRadians(0))
                .build();

        Trajectory targetZoneCTraj2 = mecanumDrive.trajectoryBuilder(targetZoneCTraj1.end(), true)
                .splineTo(new Vector2d(6, -34), Math.toRadians(0))
                .splineTo(new Vector2d(-6, -22), Math.toRadians(0))
                .build();



        //Third Trajectory - Move back to target zone
        Trajectory targetZoneATraj3 = mecanumDrive.trajectoryBuilder(targetZoneATraj2.end())
                .splineTo(new Vector2d(6, -34), Math.toRadians(0))
                .splineTo(new Vector2d(30, -42), Math.toRadians(0))
                .build();

        Trajectory targetZoneBTraj3 = mecanumDrive.trajectoryBuilder(targetZoneBTraj2.end())
                .splineTo(new Vector2d(6, -34), Math.toRadians(0))
                .splineTo(new Vector2d(54, -18), Math.toRadians(0))
                .build();

        Trajectory targetZoneCTraj3 = mecanumDrive.trajectoryBuilder(targetZoneCTraj2.end())
                .splineTo(new Vector2d(6, -34), Math.toRadians(0))
                .splineTo(new Vector2d(76, -42), Math.toRadians(0))
                .build();



        //Fourth Trajectory - Move to shooting point
        Trajectory targetZoneATraj4 = mecanumDrive.trajectoryBuilder(targetZoneATraj3.end())
                .splineTo(new Vector2d(18, -42), Math.toRadians(0))
                .build();

        Trajectory targetZoneBTraj4 = mecanumDrive.trajectoryBuilder(targetZoneBTraj3.end())
                .splineTo(new Vector2d(42, -18), Math.toRadians(0))
                .splineTo(new Vector2d(18, -42), Math.toRadians(0))
                .build();

        Trajectory targetZoneCTraj4 = mecanumDrive.trajectoryBuilder(targetZoneCTraj3.end())
                .splineTo(new Vector2d(18, -42), Math.toRadians(0))
                .build();


        //Fifth Trajectory - Park on the launch line
        Trajectory targetZoneATraj5 = mecanumDrive.trajectoryBuilder(targetZoneATraj4.end())
                .splineTo(new Vector2d(30, -42), Math.toRadians(0))
                .build();

        Trajectory targetZoneBTraj5 = mecanumDrive.trajectoryBuilder(targetZoneBTraj4.end())
                .splineTo(new Vector2d(30, -42), Math.toRadians(0))
                .build();

        Trajectory targetZoneCTraj5 = mecanumDrive.trajectoryBuilder(targetZoneCTraj4.end())
                .splineTo(new Vector2d(30, -42), Math.toRadians(0))
                .build();

        /**
         * Follow the appropriate trajectories
         */

        if (targetZone == OpenCV.targetZone.A) {
            //Move to Target Zone A
            mecanumDrive.followTrajectory(targetZoneATraj1);

            //Drop wobble goal
            dropWobbleGoal();

            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneATraj2);

            //Pick up wobble goal
            grabWobbleGoal();

            //Move back to target zone
            mecanumDrive.followTrajectory(targetZoneATraj3);

            //Drop wobble goal
            dropWobbleGoal();

            //Move to shooting point
            mecanumDrive.followTrajectory(targetZoneATraj4);

            //Shoot the rings
            shoot();

            //Park on launch line
            mecanumDrive.followTrajectory(targetZoneATraj5);
        }
        if (targetZone == OpenCV.targetZone.B) {
            //Move to Target Zone B
            mecanumDrive.followTrajectory(targetZoneBTraj1);

            //Drop wobble goal
            dropWobbleGoal();

            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneBTraj2);

            //Pick up wobble goal
            grabWobbleGoal();

            //Move back to target zone
            mecanumDrive.followTrajectory(targetZoneBTraj3);

            //Drop wobble goal
            dropWobbleGoal();

            //Move to shooting point
            mecanumDrive.followTrajectory(targetZoneATraj4);

            //Shoot the rings
            shoot();

            //Park on launch line
            mecanumDrive.followTrajectory(targetZoneATraj5);
        }
        if (targetZone == OpenCV.targetZone.C) {
            //Move to Target Zone C
            mecanumDrive.followTrajectory(targetZoneCTraj1);

            //Drop wobble goal
            dropWobbleGoal();

            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneCTraj2);

            //Pick up wobble goal
            grabWobbleGoal();

            //Move back to target zone
            mecanumDrive.followTrajectory(targetZoneBTraj3);

            //Drop wobble goal
            dropWobbleGoal();

            //Move to shooting point
            mecanumDrive.followTrajectory(targetZoneATraj4);

            //Shoot the rings
            shoot();

            //Park on launch line
            mecanumDrive.followTrajectory(targetZoneATraj5);
        }

    }

    public void dropWobbleGoal() {
        //Drop wobble goal
        claw.armDown();
        sleep(100);
        claw.open();
        sleep(100);
        claw.armUp();
        sleep(100);
    }

    public void grabWobbleGoal() {
        //Pick up wobble goal
        claw.armDown();
        sleep(100);
        claw.close();
        sleep(100);
        claw.armUp();
        sleep(100);
    }

    public void shoot() {}

    public Vector2d rotate(Vector2d point) {
        double x = point.getX();
        double y = point.getY();
        Vector2d rotatedPoint = new Vector2d(y, -x);
        return rotatedPoint;
    }
}
