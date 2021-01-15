package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.OpenCV;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;

@Autonomous
public class DropNShootRoadRunnerAuto extends FishloAutonomousProgram {

    //Back right corner is (-72, -72)
    //Back left corner is (-72, 24)
    //Front right corner is (72, -72)
    //Front left corner is (72, 24)

    //Global Variables

    public static SampleMecanumDrive mecanumDrive;
    public static Pose2d startPose;
    public volatile static Pose2d endPose;
    protected OpenCV.targetZone targetZone;

    // Target Zone A Trajectories
    Trajectory targetZoneATraj1;
    Trajectory targetZoneATraj2;
    Trajectory targetZoneATraj3;
    Trajectory targetZoneATraj4;
    Trajectory targetZoneATraj5;
    Trajectory targetZoneATraj6;

    // Target Zone B Trajectories
    Trajectory targetZoneBTraj1;
    Trajectory targetZoneBTraj2;
    Trajectory targetZoneBTraj3;
    Trajectory targetZoneBTraj4;
    Trajectory targetZoneBTraj5;
    Trajectory targetZoneBTraj6;

    // Target Zone C Trajectories
    Trajectory targetZoneCTraj1;
    Trajectory targetZoneCTraj2;
    Trajectory targetZoneCTraj3;
    Trajectory targetZoneCTraj4;
    Trajectory targetZoneCTraj5;
    Trajectory targetZoneCTraj6;

    // Pose Tracker Thread
    PoseTracker poseTracker;

    // Gives the robot object so we can access it's methods
    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    // Init
    @Override
    public void preMain() {


        // Make the SampleMecanumDrive object for RoadRunner
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-63, -49, Math.toRadians(180));
        // Set the start pose
        mecanumDrive.setPoseEstimate(startPose);

        // Start the pose tracker thread
        poseTracker = new PoseTracker(mecanumDrive, startPose);
        poseTracker.start();

        //openCV.initVision();


        // Make TrajectoryBuilderA thread
        TrajectoryBuilderA trajectoryBuilderA = new TrajectoryBuilderA();
        trajectoryBuilderA.setRobot(getRobot());

        // Make TrajectoryBuilderB thread
        TrajectoryBuilderB trajectoryBuilderB = new TrajectoryBuilderB();
        trajectoryBuilderB.setRobot(getRobot());

        // Make TrajectoryBuilderC thread
        TrajectoryBuilderC trajectoryBuilderC = new TrajectoryBuilderC();
        trajectoryBuilderC.setRobot(getRobot());

        //Start all of the threads

        trajectoryBuilderA.start();

        telemetry.addLine("Thread A started");
        telemetry.update();

        trajectoryBuilderB.start();

        telemetry.addLine("Thread B started");
        telemetry.update();

        trajectoryBuilderC.start();

        telemetry.addLine("Thread C started");
        telemetry.update();

        // Wait until all of the threads are finished
        try {
            trajectoryBuilderA.join();
            trajectoryBuilderB.join();
            trajectoryBuilderC.join();
            telemetry.addLine("Joining threads");
            telemetry.update();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // Get all of the target zone A trajectories
        ArrayList<Trajectory> targetZoneATrajList = trajectoryBuilderA.getTrajectoryList();

        targetZoneATraj1 = targetZoneATrajList.get(0);
        targetZoneATraj2 = targetZoneATrajList.get(1);
        targetZoneATraj3 = targetZoneATrajList.get(2);
        targetZoneATraj4 = targetZoneATrajList.get(3);
        targetZoneATraj5 = targetZoneATrajList.get(4);
        targetZoneATraj6 = targetZoneATrajList.get(5);

        telemetry.addLine("Target Zone A Trajectories Completed");
        telemetry.update();

        // Get all of the target zone B trajectories
        ArrayList<Trajectory> targetZoneBTrajList = trajectoryBuilderB.getTrajectoryList();

        targetZoneBTraj1 = targetZoneBTrajList.get(0);
        targetZoneBTraj2 = targetZoneBTrajList.get(1);
        targetZoneBTraj3 = targetZoneBTrajList.get(2);
        targetZoneBTraj4 = targetZoneBTrajList.get(3);
        targetZoneBTraj5 = targetZoneBTrajList.get(4);
        targetZoneBTraj6 = targetZoneBTrajList.get(5);

        telemetry.addLine("Target Zone B Trajectories Completed");
        telemetry.update();

        // Get all of the target zone C trajectories
        ArrayList<Trajectory> targetZoneCTrajList = trajectoryBuilderC.getTrajectoryList();

        targetZoneCTraj1 = targetZoneCTrajList.get(0);
        targetZoneCTraj2 = targetZoneCTrajList.get(1);
        targetZoneCTraj3 = targetZoneCTrajList.get(2);
        targetZoneCTraj4 = targetZoneCTrajList.get(3);
        targetZoneCTraj5 = targetZoneCTrajList.get(4);
        targetZoneCTraj6 = targetZoneCTrajList.get(5);
        telemetry.addLine("Target Zone C Trajectories Completed");
        telemetry.update();

        telemetry.addLine("Ready for start");
        telemetry.update();

        // Get the target zone from the webcam
        while (!isStarted()) {

            targetZone = OpenCV.targetZone.C; //openCV.getTargetZone();
            //telemetry.addData("Rings", openCV.getHeight());

        }
    }

    @Override
    public void main() {

        // Stop streaming from the webcam to save power
        // openCV.stopAll();


        // Show the target zone
        telemetry.addData("TargetZone", targetZone);

        /**
         * Follow the appropriate trajectories
         */

        telemetry.addLine("Starting Program");
        telemetry.update();
        if (targetZone == OpenCV.targetZone.A) {

            // Move to Target Zone A and drop wobble goal
            telemetry.addLine("Moving to target zone A");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj1);

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();

            // Move to shooting position
            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj2);

            // Shoot
            telemetry.addLine("Shooting");
            telemetry.update();
            shoot();

            // Move to second wobble goal
            telemetry.addLine("Moving to second wobble goal");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj3);

            // Grab wobble goal

            telemetry.addLine("Grabbing wobble goal");
            telemetry.update();
            claw.armDown();
            mecanumDrive.followTrajectory(targetZoneATraj4);
            claw.close();
            sleep(500);
            claw.armUp();

            // Move back to target zone A and drop wobble goal
            telemetry.addLine("Moving back to target zone A");
            mecanumDrive.followTrajectory(targetZoneATraj5);

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();


            // Park
            telemetry.addLine("Parking");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj6);

        }
        if (targetZone == OpenCV.targetZone.B) {

            // Move to target zone B and drop wobble goal
            telemetry.addLine("Moving to target zone B");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj1);

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();

            // Move to shooting position
            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj2);

            // Shoot rings
            telemetry.addLine("Shooting");
            telemetry.update();
            shoot();

            // Move to second wobble goal
            telemetry.addLine("Moving to second wobble goal");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj3);

            // Grab wobble goal
            telemetry.addLine("Grabbing wobble goal");
            telemetry.update();
            claw.armDown();
            mecanumDrive.followTrajectory(targetZoneBTraj4);
            claw.close();
            sleep(500);
            claw.armUp();

            // Move to target zone and drop wobble goal
            telemetry.addLine("Moving back to target zone B");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj5);

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();

            // Park
            telemetry.addLine("Parking");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj6);

        }
        if (targetZone == OpenCV.targetZone.C) {

            // Move to Target Zone C
            telemetry.addLine("Moving to target zone C");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneCTraj1);

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();

            // Move to shooting position
            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneCTraj2);

            // Shoot the rings
            telemetry.addLine("Shooting rings");
            telemetry.update();
            shoot();

            // Move to second wobble goal
            telemetry.addLine("Moving to second wobble goal");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneCTraj3);

            // Grab wobble goal
            telemetry.addLine("Grabbing wobble goal");
            telemetry.update();
            claw.armDown();
            mecanumDrive.followTrajectory(targetZoneCTraj4);
            claw.close();
            sleep(500);
            claw.armUp();

            // Move back to target zone C
            telemetry.addLine("Moving back to target zone C");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneCTraj5);

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();

            // Park
            telemetry.addLine("Parking");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneCTraj6);

        }

        // Stop the pose tracker thread
        poseTracker.stopThread();
        try {
            poseTracker.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    // Shoot function
    public void shoot() {}

    // Rotate function
    public Vector2d rotate(Vector2d point) {
        double x = point.getX();
        double y = point.getY();
        Vector2d rotatedPoint = new Vector2d(y, -x);
        return rotatedPoint;
    }
}

// Thread to build target zone A trajectories
class TrajectoryBuilderA extends Thread {
    private volatile ArrayList<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private SampleMecanumDrive mecanumDrive;
    private Pose2d startPose;
    private Robot robot;

    // Run method
    public void run() {
        try
        {
            // Build trajectories
            Trajectory targetZoneATraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
                    .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(7, -35), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneATraj1);

            Trajectory targetZoneATraj2 = mecanumDrive.trajectoryBuilder(targetZoneATraj1.end())
                    .forward(12)
                    .build();
            trajectoryList.add(targetZoneATraj2);

            Trajectory targetZoneATraj3 = mecanumDrive.trajectoryBuilder(targetZoneATraj2.end())
                    .splineToConstantHeading(new Vector2d(5, -5), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-67, 4), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneATraj3);

            Trajectory targetZoneATraj4 = mecanumDrive.trajectoryBuilder(targetZoneATraj3.end(), true)
                    .strafeLeft(8)
                    .build();
            trajectoryList.add(targetZoneATraj4);

            Trajectory targetZoneATraj5 = mecanumDrive.trajectoryBuilder(targetZoneATraj4.end(), true)
                    .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-10, -40), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneATraj5);


            Trajectory targetZoneATraj7 = mecanumDrive.trajectoryBuilder(targetZoneATraj5.end(), true)
                    .back(5)
                    .build();
            trajectoryList.add(targetZoneATraj7);

            robot.telemetry.addLine("Target Zone A Trajectories Built");
            robot.telemetry.update();


        }
        catch (Exception e)
        {
            // Throwing an exception
            System.out.println ("Exception is caught");
        }
    }

    public TrajectoryBuilderA() {
        mecanumDrive = DropNShootRoadRunnerAuto.mecanumDrive;
        startPose = DropNShootRoadRunnerAuto.startPose;
    }

    // Set the robot
    public void setRobot(Robot r) {
        robot = r;
    }

    // Get the trajectory list
    public ArrayList<Trajectory> getTrajectoryList() {
        return trajectoryList;
    }
}

// Thread to build target zone B trajectories
class TrajectoryBuilderB extends Thread {
    private volatile ArrayList<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private SampleMecanumDrive mecanumDrive;
    private Pose2d startPose;
    private Robot robot;

    public void run() {
        try
        {
            // Displaying the thread that is running
            Trajectory targetZoneBTraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
                    .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(35, -14), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneBTraj1);


            Trajectory targetZoneBTraj2 = mecanumDrive.trajectoryBuilder(targetZoneBTraj1.end())
                    .splineToConstantHeading(new Vector2d(17, -14), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(3, -35), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneBTraj2);


            Trajectory targetZoneBTraj3 = mecanumDrive.trajectoryBuilder(targetZoneBTraj2.end())
                    .splineToConstantHeading(new Vector2d(5, -5), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-56.5, 5.5), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneBTraj3);


            Trajectory targetZoneBTraj4 = mecanumDrive.trajectoryBuilder(targetZoneBTraj3.end(), true)
                    .strafeLeft(13)
                    .build();
            trajectoryList.add(targetZoneBTraj4);


            Trajectory targetZoneBTraj5 = mecanumDrive.trajectoryBuilder(targetZoneBTraj4.end(), true)
                    .splineToConstantHeading(new Vector2d(0, -5), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(28.5, -20), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneBTraj5);


            Trajectory targetZoneBTraj6 = mecanumDrive.trajectoryBuilder(targetZoneBTraj5.end())
                    .forward(15)
                    .build();
            trajectoryList.add(targetZoneBTraj6);

            robot.telemetry.addLine("Target Zone B Trajectories Built");
            robot.telemetry.update();
        }
        catch (Exception e)
        {
            // Throwing an exception
            System.out.println ("Exception is caught");
        }
    }

    public TrajectoryBuilderB() {
        mecanumDrive = DropNShootRoadRunnerAuto.mecanumDrive;
        startPose = DropNShootRoadRunnerAuto.startPose;
    }

    public void setRobot(Robot r) {
        robot = r;
    }
    public ArrayList<Trajectory> getTrajectoryList() {
        return trajectoryList;
    }
}

// Thread to build target zone C trajectories
class TrajectoryBuilderC extends Thread {
    private volatile ArrayList<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private SampleMecanumDrive mecanumDrive;
    private Pose2d startPose;
    private Robot robot;

    public void run() {
        try
        {
            // Displaying the thread that is running
            Trajectory targetZoneCTraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
                    .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(56, -40), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneCTraj1);

            Trajectory targetZoneCTraj2 = mecanumDrive.trajectoryBuilder(targetZoneCTraj1.end())
                    .forward(57)
                    .build();
            trajectoryList.add(targetZoneCTraj2);

            Trajectory targetZoneCTraj3 = mecanumDrive.trajectoryBuilder(targetZoneCTraj2.end())
                    .splineToConstantHeading(new Vector2d(-20, -40), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(180)), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneCTraj3);

            Trajectory targetZoneCTraj4 = mecanumDrive.trajectoryBuilder(targetZoneCTraj3.end(), true)
                    .strafeLeft(9)
                    .build();
            trajectoryList.add(targetZoneCTraj4);

            Trajectory targetZoneCTraj5 = mecanumDrive.trajectoryBuilder(targetZoneCTraj4.end(), true)
                    .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(43, -40), Math.toRadians(180))
                    .build();
            trajectoryList.add(targetZoneCTraj5);

            Trajectory targetZoneCTraj6 = mecanumDrive.trajectoryBuilder(targetZoneCTraj5.end())
                    .forward(40)
                    .build();
            trajectoryList.add(targetZoneCTraj6);

            robot.telemetry.addLine("Target Zone C Trajectories Built");
            robot.telemetry.update();



        }
        catch (Exception e)
        {
            // Throwing an exception
            System.out.println ("Exception is caught");
        }
    }

    public TrajectoryBuilderC() {
        mecanumDrive = DropNShootRoadRunnerAuto.mecanumDrive;
        startPose = DropNShootRoadRunnerAuto.startPose;
    }

    public void setRobot(Robot r) {
        robot = r;
    }

    public ArrayList<Trajectory> getTrajectoryList() {
        return trajectoryList;
    }
}

class PoseTracker extends Thread {
    private SampleMecanumDrive mecanumDrive;
    private boolean exit;

    public PoseTracker(SampleMecanumDrive drive, Pose2d startPose) {
        mecanumDrive = drive;
        mecanumDrive.setPoseEstimate(startPose);
        exit = false;
    }

    public void run() {
        while (!exit) {
            DropNShootRoadRunnerAuto.endPose = mecanumDrive.getPoseEstimate();
        }
    }

    public void stopThread() {
        exit = true;
    }
}