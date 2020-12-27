package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Fishlo extends Robot {

    public Fishlo(OpMode opMode) {
        super(opMode);

        putSubSystem("Drive", new Drive(this));
        putSubSystem("Gyro", new Gyro(this));
        putSubSystem("Claw", new Claw(this));
        putSubSystem("Vuforia", new Vuforia(this));
        putSubSystem("Shooter", new Shooter(this));
        putSubSystem("Intake", new Intake(this));
        putSubSystem("OpenCV", new OpenCV(this));
    }

}
