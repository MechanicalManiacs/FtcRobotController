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

    SampleMecanumDrive mecanumDrive;
    Pose2d startPose;
    public static Pose2d endPose;
    protected OpenCV.targetZone targetZone;

    Trajectory targetZoneATraj1;
    Trajectory targetZoneATraj2;
    Trajectory targetZoneATraj3;
    Trajectory targetZoneATraj4;
    Trajectory targetZoneATraj5;
    Trajectory targetZoneATraj6;

    Trajectory targetZoneBTraj1;
    Trajectory targetZoneBTraj2;
    Trajectory targetZoneBTraj3;
    Trajectory targetZoneBTraj4;
    Trajectory targetZoneBTraj5;

    Trajectory targetZoneCTraj1;
    Trajectory targetZoneCTraj2;
    Trajectory targetZoneCTraj3;
    Trajectory targetZoneCTraj4;
    Trajectory targetZoneCTraj5;

    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void preMain() {

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-63, -49, Math.toRadians(180));
        mecanumDrive.setPoseEstimate(startPose);
        //openCV.initVision();


        // -9, -3
        //First Trajectory - Move to Target Zone
        targetZoneATraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(7, -35), Math.toRadians(0))
                .build();

        targetZoneBTraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(34, -14), Math.toRadians(0))
                .build();

        targetZoneCTraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(57, -39), Math.toRadians(0))
                .build();
        telemetry.addLine("Built first trajectories");
        telemetry.update();



        //Second Trajectory - Move to shooting position
        targetZoneATraj2 = mecanumDrive.trajectoryBuilder(targetZoneATraj1.end())
                .forward(12)
                .build();

        targetZoneBTraj2 = mecanumDrive.trajectoryBuilder(targetZoneBTraj1.end())
                .splineToConstantHeading(new Vector2d(17, -14), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, -40), Math.toRadians(0))
                .build();

        targetZoneCTraj2 = mecanumDrive.trajectoryBuilder(targetZoneCTraj1.end())
                .forward(57)
                .build();

        telemetry.addLine("Built second trajectories");
        telemetry.update();



        //Third Trajectory - Move to second wobble goal
        targetZoneATraj3 = mecanumDrive.trajectoryBuilder(targetZoneATraj2.end())
                .splineToConstantHeading(new Vector2d(-30, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-63.2, -7), Math.toRadians(0))
                .build();

        targetZoneBTraj3 = mecanumDrive.trajectoryBuilder(targetZoneBTraj2.end())
                .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-49, -5), Math.toRadians(0))
                .build();

        targetZoneCTraj3 = mecanumDrive.trajectoryBuilder(targetZoneCTraj2.end())
                .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-49, -5), Math.toRadians(0))
                .build();
        telemetry.addLine("Built third trajectories");
        telemetry.update();




        //Fourth Trajectory - Move to target zone
        targetZoneATraj4 = mecanumDrive.trajectoryBuilder(targetZoneATraj3.end(), true)
                .strafeLeft(8)
                .build();

        targetZoneBTraj4 = mecanumDrive.trajectoryBuilder(targetZoneBTraj3.end(), true)
                .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(34, -14), Math.toRadians(0))
                .build();

        targetZoneCTraj4 = mecanumDrive.trajectoryBuilder(targetZoneBTraj3.end(), true)
                .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(34, -14), Math.toRadians(0))
                .build();
        telemetry.addLine("Built fourth trajectories");
        telemetry.update();



        //Fifth Trajectory - Park on the launch line

        targetZoneATraj5 = mecanumDrive.trajectoryBuilder(targetZoneATraj3.end(), true)
                .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-15, -45), Math.toRadians(0))
                .build();
        targetZoneBTraj5 = mecanumDrive.trajectoryBuilder(targetZoneBTraj4.end())
                .forward(22)
                .build();

        targetZoneCTraj5 = mecanumDrive.trajectoryBuilder(targetZoneCTraj4.end())
                .forward(45)
                .build();
        telemetry.addLine("Built fifth trajectories");
        telemetry.update();

        targetZoneATraj6 = mecanumDrive.trajectoryBuilder(targetZoneATraj3.end(), true)
                .back(5)
                .build();

        telemetry.addLine("Ready");
        telemetry.update();
        while (!isStarted()) {

            targetZone = OpenCV.targetZone.A; //openCV.getTargetZone();
            telemetry.addData("TargetZone", targetZone);
            //telemetry.addData("Rings", openCV.getHeight());

        }
    }

    @Override
    public void main() {
        // openCV.stopAll();

        /**
         * Initialize Trajectories
         */




        /**
         * Follow the appropriate trajectories
         */

        telemetry.addLine("Starting Program");
        telemetry.update();
        if (targetZone == OpenCV.targetZone.A) {

            telemetry.addLine("Moving to target zone A");
            telemetry.update();
            //Move to Target Zone A
            mecanumDrive.followTrajectory(targetZoneATraj1);

            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            telemetry.addLine("Dropping Wobble Goal");
            telemetry.update();
            //Drop wobble goal
            dropWobbleGoal();

            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            //Move to shooting position
            mecanumDrive.followTrajectory(targetZoneATraj2);

            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            telemetry.addLine("Shooting");
            telemetry.update();
            //Shoot
            shoot();

            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            telemetry.addLine("Moving to second wobble goal");
            telemetry.update();
            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneATraj3);

            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }



            telemetry.addLine("Grabbing wobble goal");
            //Grab wobble goal
            claw.armDown();

            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneATraj4);

            claw.close();
            sleep(500);
            claw.armUp();
            sleep(200);

            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }
//
            telemetry.addLine("Moving back to target zone A");
            //Move to target zone
            mecanumDrive.followTrajectory(targetZoneATraj5);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }
//
            telemetry.addLine("Dropping to target zone A");
            //Drop wobble goal
            dropWobbleGoal();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }
//
            telemetry.addLine("Parked");
            mecanumDrive.followTrajectory(targetZoneATraj6);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }
            telemetry.update();

        }
        if (targetZone == OpenCV.targetZone.B) {
            //Move to Target Zone B
            mecanumDrive.followTrajectory(targetZoneBTraj1);

            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }
            //Drop wobble goal
            dropWobbleGoal();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Move to shooting position
            mecanumDrive.followTrajectory(targetZoneBTraj2);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Pick up wobble goal
            shoot();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneBTraj3);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Grab wobble goal
            grabWobbleGoal();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Move to target zone
            mecanumDrive.followTrajectory(targetZoneBTraj4);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Drop wobble goal
            dropWobbleGoal();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            mecanumDrive.followTrajectory(targetZoneBTraj5);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }
        }
        if (targetZone == OpenCV.targetZone.C) {
            //Move to Target Zone C
            mecanumDrive.followTrajectory(targetZoneCTraj1);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Drop wobble goal
            dropWobbleGoal();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Move to shooting position
            mecanumDrive.followTrajectory(targetZoneCTraj2);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Pick up wobble goal
            shoot();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneCTraj3);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Grab wobble goal
            grabWobbleGoal();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Move to target zone
            mecanumDrive.followTrajectory(targetZoneCTraj4);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            //Drop wobble goal
            dropWobbleGoal();
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }

            mecanumDrive.followTrajectory(targetZoneCTraj5);
            if (isStopRequested()) {
                endPose = mecanumDrive.getPoseEstimate();
            }
        }

        endPose = mecanumDrive.getPoseEstimate();

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
        sleep(200);
        claw.close();
        sleep(500);
        claw.armUp();
        sleep(200);
    }

    public void shoot() {}

    public Vector2d rotate(Vector2d point) {
        double x = point.getX();
        double y = point.getY();
        Vector2d rotatedPoint = new Vector2d(y, -x);
        return rotatedPoint;
    }
}
