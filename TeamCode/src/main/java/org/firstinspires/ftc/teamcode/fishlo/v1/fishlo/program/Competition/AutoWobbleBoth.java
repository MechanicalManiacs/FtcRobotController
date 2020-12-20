package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Utility.PID;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class AutoWobbleBoth extends FishloAutonomousProgram {
    //Create the variable targetZone to store the targetZone value from vision
    protected char targetZone;

    //Create the variables for PID constants
    protected final double Kp = 0.1;
    protected final double HEADING_THRESHOLD = 1;
    protected final double Kd = 0; //425;
    protected final double ROBOT_SPEED = 0.75;
    protected final double PARK_TIME = 29;
    ElapsedTime timer;


    //Build the robot
    @Override
    public Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    //This method is for code that needs to run during the init phase
    @Override
    public void preMain() {
        //Initialize the imu
        gyro.initGyro();
        shooter.resetPusher();
        timer = new ElapsedTime();

        //Reset claw and arm to starting position
//        claw.close();
        //Timer for vision

        //Make sure that the telemetry clears when printing on the screen
        telemetry.setAutoClear(true);
        //Find the targetZone based on the starter stack
        while (!isStopRequested() && !gyro.isCalibrated()) {
            sleep(50);
            idle();
        }
        gyro.getHeading();
        telemetry.addData("Gyro", gyro.getCalibrationStatus());
        telemetry.update();
        while (!isStarted()) {

            targetZone = vision.getTargetZone();

        }


    }

    //This method is for code that needs to run after start is pressed.
    @Override
    public void main() {
        timer.reset();

        //Move wobble goal to target zone A
        if (targetZone == 'A') {
            telemetry.addData("Main", "Driving to Target Zone A");
            telemetry.update();
            //Status update: The robot is driving to the target zone
            telemetry.addData("Main", "Driving - P: 55 in, S: 0.5");
            telemetry.update();
            //Drives to position (68 inches forward at 0.5 power)
            drive.moveToPosition(73, ROBOT_SPEED);

            //Drops the wobble goal
            claw.armDown();
            claw.open();
            sleep(100);

            claw.armUp();
            sleep(100);

            telemetry.addData("Main", "Strafing - P:-12 in, S: 0.4");
            telemetry.update();

            drive.strafeToPosition(-57, ROBOT_SPEED);

            drive.moveToPosition(-62, ROBOT_SPEED);

            claw.armDown();
            sleep(100);

            drive.strafeToPosition(8, ROBOT_SPEED);

            sleep(20);
            claw.close();
            sleep(100);

            claw.armUp();

            drive.moveToPosition(58, ROBOT_SPEED);

            drive.strafeToPosition(36, ROBOT_SPEED);

            claw.armDown();
            sleep(80 );

            claw.open();
            sleep(100);

            claw.armUp();

            drive.strafeToPosition(-5, ROBOT_SPEED);

            shooter.startShooter();
            sleep(1000);
            drive.turnWithEncoder(1900, ROBOT_SPEED);
            drive.moveToPosition(10, ROBOT_SPEED);
            sleep(20);
            shooter.shoot();
            sleep(500);
            shooter.resetPusher();
            sleep(800);
            if (timer.seconds() > PARK_TIME) {
                drive.moveToPosition(-10, 1);
                shooter.stopShooter();
            }
            shooter.shoot();
            sleep(500);
            shooter.resetPusher();
            sleep(800);
            if (timer.seconds() > PARK_TIME) {
                drive.moveToPosition(-10, 1);
                shooter.stopShooter();
            }
            shooter.shoot();
            sleep(500);
            shooter.resetPusher();
            sleep(800);

            drive.moveToPosition(-10, 1);
            shooter.stopShooter();
            sleep(500);

        }
        //Move wobble goal to target zone B
        else if (targetZone == 'B') {
            telemetry.addData("Main", "Driving to Target Zone B");
            telemetry.update();
            //Status update: The robot is driving to the target zone
            telemetry.addData("Main", "Driving - P: 55 in, S: 0.5");
            telemetry.update();
            drive.moveToPosition(2, ROBOT_SPEED);
            drive.strafeToPosition(11, ROBOT_SPEED);
            //Drives to position (68 inches forward at 0.5 power)
            drive.moveToPosition(98, ROBOT_SPEED);

            drive.strafeToPosition(-40, ROBOT_SPEED);

            //Drops the wobble goal
            claw.armDown();
            claw.open();
            sleep(100);

            claw.armUp();
            sleep(100);

            telemetry.addData("Main", "Strafing - P:-12 in, S: 0.4");
            telemetry.update();


            drive.strafeToPosition(-40, 0.6);

            drive.moveToPosition(-85, ROBOT_SPEED);

            claw.armDown();
            sleep(100);

            drive.strafeToPosition(19, ROBOT_SPEED);

            sleep(200);
            claw.close();
            sleep(400);

            claw.armUp();

            drive.moveToPosition(77, ROBOT_SPEED);


            claw.armDown();
            sleep(80 );

            claw.open();
            sleep(100);

            claw.armUp();
            shooter.startShooter();

            drive.strafeToPosition(-5, ROBOT_SPEED);

            drive.turnWithEncoder(1300, ROBOT_SPEED);
            shooter.shoot();
            sleep(500);
            shooter.resetPusher();
            sleep(800);
            if (timer.seconds() > PARK_TIME) {
//                drive.moveToPosition(-10, 1);
                shooter.stopShooter();
            }
            shooter.shoot();
            sleep(500);
            shooter.resetPusher();
            sleep(800);
            if (timer.seconds() > PARK_TIME) {
//                drive.moveToPosition(-10, 1);
                shooter.stopShooter();
            }
            shooter.shoot();
            sleep(500);
            shooter.resetPusher();
            sleep(800);

            drive.moveToPosition(-10, 1);
            shooter.stopShooter();
            sleep(500);
        }
        //Move wobble goal to target zone C
        else if (targetZone == 'C') {
            telemetry.addData("Main", "Driving to Target Zone C");
            telemetry.update();
            //Status update: The robot is driving to the target zone
            telemetry.addData("Main", "Driving - P: 55 in, S: 0.5");
            telemetry.update();
            //Drives to position (68 inches forward at 0.5 power)
            drive.moveToPosition(110, 1);

            drive.strafeToPosition(-10, 1);

            //Drops the wobble goal
            claw.armDown();
            sleep(100);
            claw.open();
            sleep(100);

            claw.armUp();
            sleep(100);

            telemetry.addData("Main", "Strafing - P:-12 in, S: 0.4");
            telemetry.update();

            drive.strafeToPosition(-57, 1);

            drive.moveToPosition(-100, 1);

            claw.armDown();
            sleep(100);

            drive.strafeToPosition(12, 1);





            claw.close();
            sleep(50);

            claw.armUp();
            sleep(100);

            drive.moveToPosition(100, 1);

            drive.strafeToPosition(34, 1);

            claw.armDown();
            sleep(100);

            claw.open();
            sleep(50);

            claw.armUp();
            sleep(100);

            drive.strafeToPosition(-5, 1);

            shooter.startShooter();
            sleep(1000);
            drive.turnWithEncoder(1900, 1);
            drive.moveToPosition(48, 1);
            shooter.shoot();
            sleep(500);
            shooter.resetPusher();
            sleep(1000);
            shooter.shoot();
            sleep(500);
            shooter.resetPusher();
            sleep(800);

            drive.moveToPosition(-35, 1);
            shooter.stopShooter();
            sleep(500);

        }
        //Clears the screen of all telemetry updates
        telemetry.clear();
        //Status update: The program is complete
        telemetry.addData("Main", "Program Complete");
        telemetry.update();
    }
}