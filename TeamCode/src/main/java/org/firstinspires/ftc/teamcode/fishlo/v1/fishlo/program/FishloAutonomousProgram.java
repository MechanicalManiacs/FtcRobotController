package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.*;
import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class FishloAutonomousProgram extends AutonomousProgram {
    protected Claw claw;
    protected Drive drive;
    protected Gyro gyro;
    protected Intake intake;
    protected Shooter shooter;
    protected OpenCV openCV;

    @Override
    protected Robot buildRobot() {
        Fishlo fishlo = new Fishlo(this);

        drive = (Drive) fishlo.getSubSystem("Drive");
        claw = (Claw) fishlo.getSubSystem("Claw");
        gyro = (Gyro) fishlo.getSubSystem("Gyro");
        intake = (Intake) fishlo.getSubSystem("Intake");
        shooter = (Shooter) fishlo.getSubSystem("Shooter");
        openCV = (OpenCV) fishlo.getSubSystem("OpenCV");

        return fishlo;
    }

    @Override
    public void main() {}

    @Override
    public void preMain() {}
}
