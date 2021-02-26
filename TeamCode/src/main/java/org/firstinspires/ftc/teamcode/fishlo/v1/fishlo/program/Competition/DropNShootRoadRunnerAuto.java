package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
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

    //Global Variables

    public static SampleMecanumDrive mecanumDrive;
    public static Pose2d startPose;
    public static boolean autoEnded = false;
    public volatile static Pose2d endPose;
    protected OpenCV.targetZone targetZone;

    // Target Zone A Trajectories
    private Trajectory targetZoneATraj1;
    private Trajectory targetZoneATraj2;
    private Trajectory targetZoneATraj3;
    private Trajectory targetZoneATraj4;
    private Trajectory targetZoneATraj5;
    private Trajectory targetZoneATraj6;

    // Target Zone B Trajectories
    private Trajectory targetZoneBTraj1;
    private Trajectory targetZoneBTraj2;
    private Trajectory targetZoneBTraj3;
    private Trajectory targetZoneBTraj4;
    private Trajectory targetZoneBTraj5;
    private Trajectory targetZoneBTraj6;
    private Trajectory targetZoneBTraj7;
    private Trajectory targetZoneBTraj8;
    private Trajectory targetZoneBTraj9;
    private Trajectory targetZoneBTraj10;
    private Trajectory targetZoneBTraj11;

    // Target Zone C Trajectories
    private Trajectory targetZoneCTraj1;
    private Trajectory targetZoneCTraj2;
    private Trajectory targetZoneCTraj3;
    private Trajectory targetZoneCTraj4;
    private Trajectory targetZoneCTraj5;
    private Trajectory targetZoneCTraj6;
    private Trajectory targetZoneCTraj7;

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


        autoEnded = false;
        // Make the SampleMecanumDrive object for RoadRunner
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-63, -40, Math.toRadians(180));
        // Set the start pose
        mecanumDrive.setPoseEstimate(startPose);

        // Start the pose tracker thread
        poseTracker = new PoseTracker(mecanumDrive, startPose);
        poseTracker.start();

        openCV.initVision();
        
        targetZoneATraj1 = ThreadBuilder.targetZoneATraj1;
        targetZoneATraj2 = ThreadBuilder.targetZoneATraj2;
        targetZoneATraj3 = ThreadBuilder.targetZoneATraj3;
        targetZoneATraj4 = ThreadBuilder.targetZoneATraj4;
        targetZoneATraj5 = ThreadBuilder.targetZoneATraj5;
        targetZoneATraj6 = ThreadBuilder.targetZoneATraj6;

        targetZoneBTraj1 = ThreadBuilder.targetZoneBTraj1;
        targetZoneBTraj2 = ThreadBuilder.targetZoneBTraj2;
        targetZoneBTraj3 = ThreadBuilder.targetZoneBTraj3;
        targetZoneBTraj4 = ThreadBuilder.targetZoneBTraj4;
        targetZoneBTraj5 = ThreadBuilder.targetZoneBTraj5;
        targetZoneBTraj6 = ThreadBuilder.targetZoneBTraj6;
        targetZoneBTraj7 = ThreadBuilder.targetZoneBTraj7;
        targetZoneBTraj8 = ThreadBuilder.targetZoneBTraj8;
        targetZoneBTraj9 = ThreadBuilder.targetZoneBTraj9;
        targetZoneBTraj10 = ThreadBuilder.targetZoneBTraj10;
        targetZoneBTraj11 = ThreadBuilder.targetZoneBTraj11;
        

        targetZoneCTraj1 = ThreadBuilder.targetZoneCTraj1;
        targetZoneCTraj2 = ThreadBuilder.targetZoneCTraj2;
        targetZoneCTraj3 = ThreadBuilder.targetZoneCTraj3;
        targetZoneCTraj4 = ThreadBuilder.targetZoneCTraj4;
        targetZoneCTraj5 = ThreadBuilder.targetZoneCTraj5;
        targetZoneCTraj6 = ThreadBuilder.targetZoneCTraj6;
        targetZoneCTraj7 = ThreadBuilder.targetZoneCTraj7;




        telemetry.addLine("Ready for start");
        telemetry.update();

        // Get the target zone from the webcam
        while (!isStarted()) {

            targetZone = openCV.getTargetZone();
            telemetry.clear();
            telemetry.addData("Target Zone: ", targetZone);
            telemetry.addData("Rings: ", openCV.getHeight());
            telemetry.update();
        }
    }

    @Override
    public void main() {

        autoEnded = false;
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

            shooter.startShooterAuto(1);
            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(1000);
            claw.armUp();

            // Move to shooting position
            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj2);

            claw.close();
            claw.armDown();
            sleep(500);
            // Shoot
            telemetry.addLine("Shooting");
            telemetry.update();
            shoot();
            shooter.stopShooter();
            claw.armUp();
            sleep(500);
            claw.open();


            // Move to second wobble goal
            telemetry.addLine("Moving to second wobble goal");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj3);

            // Grab wobble goal
            mecanumDrive.turn(Math.toRadians(180));


            telemetry.addLine("Grabbing wobble goal");
            telemetry.update();
            claw.armDown();;
//            mecanumDrive.followTrajectory(targetZoneATraj4);
            claw.close();
            sleep(1000);
            claw.armUp();

            mecanumDrive.turn(Math.toRadians(180));

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
            shooter.startShooterAuto(1);
            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj1);

            intake.intakeRelease();
            claw.close();
            // Shoot rings
            telemetry.addLine("Shooting");
            telemetry.update();
            shoot();
            shooter.stopShooter();



            // Move to shooting position
            telemetry.addLine("Moving to target zone b");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj2);

            mecanumDrive.turn(Math.toRadians(180));

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();


            mecanumDrive.followTrajectory(targetZoneBTraj7);


            claw.open();
            claw.armDown();
            mecanumDrive.followTrajectory(targetZoneBTraj8);
            telemetry.addLine("Grabbing wobble goal");
            telemetry.update();
            claw.close();
            sleep(500);
            claw.armUp();
//
            mecanumDrive.followTrajectory(targetZoneBTraj9);

            mecanumDrive.followTrajectory(targetZoneBTraj10);

            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();

            mecanumDrive.followTrajectory(targetZoneBTraj11);


        }
        if (targetZone == OpenCV.targetZone.C) {

            shooter.startShooterAuto(1);
            // Move to shooting position
            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneCTraj1);

            claw.close();
            // Shoot rings
            telemetry.addLine("Shooting");
            telemetry.update();
            shoot();
            shooter.stopShooter();

            // Move to Target Zone C
            telemetry.addLine("Moving to target zone C");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneCTraj2);

            intake.intakeRelease();
            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();

            shooter.startShooterAuto(1);
            // Move to second wobble goal
            telemetry.addLine("Moving to wrings");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneCTraj3);
            mecanumDrive.followTrajectory(targetZoneCTraj4);
            mecanumDrive.followTrajectory(targetZoneCTraj5);
            intake.startIntake();
            mecanumDrive.followTrajectory(targetZoneCTraj6);


            sleep(3000);
            intake.stopIntake();
            mecanumDrive.followTrajectory(targetZoneCTraj7);
            claw.close();
            // Shoot rings
            telemetry.addLine("Shooting");
            telemetry.update();
            shooter.shootAuto(1.5);
            sleep(4500);
            shooter.stopShooter();
            claw.open();

            PoseStorage.currentPose = mecanumDrive.getPoseEstimate();

//
//            // Grab wobble goal
//            telemetry.addLine("Grabbing wobble goal");
//            telemetry.update();
//            claw.armDown();
//            mecanumDrive.followTrajectory(targetZoneCTraj4);
//            claw.close();
//            sleep(1000);
//            claw.armUp();
//
//            // Move back to target zone C
//            telemetry.addLine("Moving back to target zone C");
//            telemetry.update();
//            mecanumDrive.followTrajectory(targetZoneCTraj5);
//
//            // Drop wobble goal
//            telemetry.addLine("Dropping wobble goal");
//            telemetry.update();
//            claw.armDown();
//            sleep(200);
//            claw.open();
//            sleep(500);
//            claw.armUp();
//
//            // Park
//            telemetry.addLine("Parking");
//            telemetry.update();
//            mecanumDrive.followTrajectory(targetZoneCTraj6);

        }

//        intake.intakeRelease();

        // Stop the pose tracker thread
        poseTracker.stopThread();
        try {
            poseTracker.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        autoEnded = true;

    }

    // Shoot function
    public void shoot() {

        shooter.shootAuto(0.15);
        sleep(3000);
        shooter.stopPusher();

    }

    public void shoot2() {

        shooter.shootAuto(0.2);
        sleep(1000);
        shooter.stopPusher();

    }


    // Rotate function
    public Vector2d rotate(Vector2d point) {
        double x = point.getX();
        double y = point.getY();
        Vector2d rotatedPoint = new Vector2d(y, -x);
        return rotatedPoint;
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