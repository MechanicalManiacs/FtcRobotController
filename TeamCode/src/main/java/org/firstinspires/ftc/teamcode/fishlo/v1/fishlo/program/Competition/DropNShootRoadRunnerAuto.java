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
    Trajectory targetZoneBTraj6;

    Trajectory targetZoneCTraj1;
    Trajectory targetZoneCTraj2;
    Trajectory targetZoneCTraj3;
    Trajectory targetZoneCTraj4;
    Trajectory targetZoneCTraj5;

    PoseTracker poseTracker;

    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void preMain() {

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-63, -49, Math.toRadians(180));

        poseTracker = new PoseTracker(mecanumDrive, startPose);
        poseTracker.start();

        mecanumDrive.setPoseEstimate(startPose);
        //openCV.initVision();


        // -9, -3
        //First Trajectory - Move to Target Zone
        //Second Trajectory - Move to shooting position
        //Third Trajectory - Move to second wobble goal
        //Fourth Trajectory - Move to target zone
        //Fifth Trajectory - Park on the launch line

        TrajectoryBuilderA trajectoryBuilderA = new TrajectoryBuilderA();
        trajectoryBuilderA.setDrive(mecanumDrive);
        trajectoryBuilderA.setStartPose(startPose);
        trajectoryBuilderA.setRobot(getRobot());

        TrajectoryBuilderB trajectoryBuilderB = new TrajectoryBuilderB();
        trajectoryBuilderB.setDrive(mecanumDrive);
        trajectoryBuilderB.setStartPose(startPose);
        trajectoryBuilderB.setRobot(getRobot());

        TrajectoryBuilderC trajectoryBuilderC = new TrajectoryBuilderC();
        trajectoryBuilderC.setDrive(mecanumDrive);
        trajectoryBuilderC.setStartPose(startPose);
        trajectoryBuilderC.setRobot(getRobot());

        trajectoryBuilderA.start();


        telemetry.addLine("Thread A started");
        telemetry.update();

        trajectoryBuilderB.start();

        telemetry.addLine("Thread B started");
        telemetry.update();


        trajectoryBuilderC.start();

        telemetry.addLine("Thread C started");
        telemetry.update();

        try {
            trajectoryBuilderA.join();
            trajectoryBuilderB.join();
            trajectoryBuilderC.join();
            telemetry.addLine("Joining threads");
            telemetry.update();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }



        ArrayList<Trajectory> targetZoneATrajList = trajectoryBuilderA.getTrajectoryList();

        targetZoneATraj1 = targetZoneATrajList.get(0);
        targetZoneATraj2 = targetZoneATrajList.get(1);
        targetZoneATraj3 = targetZoneATrajList.get(2);
        targetZoneATraj4 = targetZoneATrajList.get(3);
        targetZoneATraj5 = targetZoneATrajList.get(4);
        targetZoneATraj6 = targetZoneATrajList.get(5);

        telemetry.addLine("Target Zone A Trajectories Built");
        telemetry.update();

    //First Trajectory - Move to Target Zone
    //Second Trajectory - Move to shooting position
    //Third Trajectory - Move to second wobble goal
    //Fourth Trajectory - Move to target zone
    //Fifth Trajectory - Park on the launch line



        ArrayList<Trajectory> targetZoneBTrajList = trajectoryBuilderB.getTrajectoryList();

        targetZoneBTraj1 = targetZoneBTrajList.get(0);
        targetZoneBTraj2 = targetZoneBTrajList.get(1);
        targetZoneBTraj3 = targetZoneBTrajList.get(2);
        targetZoneBTraj4 = targetZoneBTrajList.get(3);
        targetZoneBTraj5 = targetZoneBTrajList.get(4);
        targetZoneBTraj6 = targetZoneBTrajList.get(5);
        telemetry.addLine("Target Zone B Trajectories Built");
        telemetry.update();




        ArrayList<Trajectory> targetZoneCTrajList = trajectoryBuilderB.getTrajectoryList();

        targetZoneBTraj1 = targetZoneCTrajList.get(0);
        targetZoneBTraj2 = targetZoneCTrajList.get(1);
        targetZoneBTraj3 = targetZoneCTrajList.get(2);
        targetZoneBTraj4 = targetZoneCTrajList.get(3);
        targetZoneBTraj5 = targetZoneCTrajList.get(4);
        targetZoneBTraj6 = targetZoneCTrajList.get(5);
        telemetry.addLine("Target Zone C Trajectories Built");
        telemetry.update();

        telemetry.addLine("Ready");
        telemetry.update();
        while (!isStarted()) {

            targetZone = OpenCV.targetZone.A; //openCV.getTargetZone();
            //telemetry.addData("Rings", openCV.getHeight());

        }
    }

    @Override
    public void main() {
        // openCV.stopAll();


        telemetry.addData("TargetZone", targetZone);

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


            telemetry.addLine("Dropping Wobble Goal");
            telemetry.update();
            //Drop wobble goal
            dropWobbleGoal();

            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            //Move to shooting position
            mecanumDrive.followTrajectory(targetZoneATraj2);




            telemetry.addLine("Shooting");
            telemetry.update();
            //Shoot
            shoot();



            telemetry.addLine("Moving to second wobble goal");
            telemetry.update();
            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneATraj3);





            telemetry.addLine("Grabbing wobble goal");
            //Grab wobble goal
            claw.armDown();



            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneATraj4);



            claw.close();
            sleep(500);



            claw.armUp();
            sleep(200);


//
            telemetry.addLine("Moving back to target zone A");
            //Move to target zone
            mecanumDrive.followTrajectory(targetZoneATraj5);

//
            telemetry.addLine("Dropping to target zone A");
            //Drop wobble goal
            dropWobbleGoal();

//
            telemetry.addLine("Parked");
            mecanumDrive.followTrajectory(targetZoneATraj6);

            telemetry.update();

        }
        if (targetZone == OpenCV.targetZone.B) {
            //Move to Target Zone B
            mecanumDrive.followTrajectory(targetZoneBTraj1);


            //Drop wobble goal
            dropWobbleGoal();


            //Move to shooting position
            mecanumDrive.followTrajectory(targetZoneBTraj2);


            //Pick up wobble goal
            shoot();


            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneBTraj3);


            //Grab wobble goal
            claw.armDown();



            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneBTraj4);



            claw.close();
            sleep(500);



            claw.armUp();
            sleep(200);




            //Move to target zone
            mecanumDrive.followTrajectory(targetZoneBTraj5);


            //Drop wobble goal
            dropWobbleGoal();


            mecanumDrive.followTrajectory(targetZoneBTraj6);

        }
        if (targetZone == OpenCV.targetZone.C) {
            //Move to Target Zone C
            mecanumDrive.followTrajectory(targetZoneCTraj1);


            //Drop wobble goal
            dropWobbleGoal();


            //Move to shooting position
            mecanumDrive.followTrajectory(targetZoneCTraj2);


            //Pick up wobble goal
            shoot();


            //Move to second wobble goal
            mecanumDrive.followTrajectory(targetZoneCTraj3);


            //Grab wobble goal
            grabWobbleGoal();


            //Move to target zone
            mecanumDrive.followTrajectory(targetZoneCTraj4);


            //Drop wobble goal
            dropWobbleGoal();


            mecanumDrive.followTrajectory(targetZoneCTraj5);

        }

        poseTracker.stopThread();

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

class TrajectoryBuilderA extends Thread {
    private volatile ArrayList<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private SampleMecanumDrive mecanumDrive;
    private Pose2d startPose;
    private Robot robot;

    public void run() {
        try
        {
            // Displaying the thread that is running
            Trajectory targetZoneATraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
                    .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(7, -35), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneATraj1);

            Trajectory targetZoneATraj2 = mecanumDrive.trajectoryBuilder(targetZoneATraj1.end())
                    .forward(12)
                    .build();
            trajectoryList.add(targetZoneATraj2);

            Trajectory targetZoneATraj3 = mecanumDrive.trajectoryBuilder(targetZoneATraj2.end())
                    .splineToConstantHeading(new Vector2d(5, -5), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-56, -3.5), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneATraj3);

            Trajectory targetZoneATraj4 = mecanumDrive.trajectoryBuilder(targetZoneATraj3.end(), true)
                    .strafeLeft(8)
                    .build();
            trajectoryList.add(targetZoneATraj4);

            Trajectory targetZoneATraj5 = mecanumDrive.trajectoryBuilder(targetZoneATraj4.end(), true)
                    .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-15, -45), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneATraj5);


            Trajectory targetZoneATraj6 = mecanumDrive.trajectoryBuilder(targetZoneATraj5.end(), true)
                    .back(5)
                    .build();
            trajectoryList.add(targetZoneATraj6);

            robot.telemetry.addLine("Target Zone A Completed");
            robot.telemetry.update();


        }
        catch (Exception e)
        {
            // Throwing an exception
            System.out.println ("Exception is caught");
        }
    }

    public void setDrive(SampleMecanumDrive drive) {
        mecanumDrive = drive;
    }

    public void setStartPose(Pose2d pose) {
        startPose = pose;
    }

    public void setRobot(Robot r) {
        robot = r;
    }

    public ArrayList<Trajectory> getTrajectoryList() {
        return trajectoryList;
    }
}

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
                    .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(30, -14), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneBTraj1);


            Trajectory targetZoneBTraj2 = mecanumDrive.trajectoryBuilder(targetZoneBTraj1.end())
                    .splineToConstantHeading(new Vector2d(17, -14), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(5, -35), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneBTraj2);


            Trajectory targetZoneBTraj3 = mecanumDrive.trajectoryBuilder(targetZoneBTraj2.end())
                    .splineToConstantHeading(new Vector2d(0, -5), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-56, -6.5), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneBTraj3);


            Trajectory targetZoneBTraj4 = mecanumDrive.trajectoryBuilder(targetZoneBTraj3.end(), true)
                    .strafeLeft(8)
                    .build();
            trajectoryList.add(targetZoneBTraj4);


            Trajectory targetZoneBTraj5 = mecanumDrive.trajectoryBuilder(targetZoneBTraj4.end(), true)
                    .splineToConstantHeading(new Vector2d(0, -5), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(25, -20), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneBTraj5);


            Trajectory targetZoneBTraj6 = mecanumDrive.trajectoryBuilder(targetZoneBTraj5.end())
                    .forward(23)
                    .build();
            trajectoryList.add(targetZoneBTraj6);

            robot.telemetry.addLine("Target Zone B Completed");
            robot.telemetry.update();
        }
        catch (Exception e)
        {
            // Throwing an exception
            System.out.println ("Exception is caught");
        }
    }

    public void setDrive(SampleMecanumDrive drive) {
        mecanumDrive = drive;
    }

    public void setStartPose(Pose2d pose) {
        startPose = pose;
    }

    public void setRobot(Robot r) {
        robot = r;
    }
    public ArrayList<Trajectory> getTrajectoryList() {
        return trajectoryList;
    }
}

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
                    .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(57, -39), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneCTraj1);

            Trajectory targetZoneCTraj2 = mecanumDrive.trajectoryBuilder(targetZoneCTraj1.end())
                    .forward(57)
                    .build();
            trajectoryList.add(targetZoneCTraj2);

            Trajectory targetZoneCTraj3 = mecanumDrive.trajectoryBuilder(targetZoneCTraj2.end())
                    .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-49, -5), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneCTraj3);

            Trajectory targetZoneCTraj4 = mecanumDrive.trajectoryBuilder(targetZoneCTraj3.end(), true)
                    .splineToConstantHeading(new Vector2d(-24, -5), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(34, -14), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneCTraj4);

            Trajectory targetZoneCTraj5 = mecanumDrive.trajectoryBuilder(targetZoneCTraj4.end())
                    .forward(45)
                    .build();
            trajectoryList.add(targetZoneCTraj5);

            robot.telemetry.addLine("Target Zone C Completed");
            robot.telemetry.update();



        }
        catch (Exception e)
        {
            // Throwing an exception
            System.out.println ("Exception is caught");
        }
    }

    public void setDrive(SampleMecanumDrive drive) {
        mecanumDrive = drive;
    }

    public void setStartPose(Pose2d pose) {
        startPose = pose;
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