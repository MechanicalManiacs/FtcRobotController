package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Battery;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Claw;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Drive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Fishlo;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Gyro;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.OpenCV;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Shooter;
import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class FishloAutonomousProgram extends AutonomousProgram {
    public static Claw claw;
    protected Drive drive;
    protected Gyro gyro;
    protected Intake intake;
    protected Shooter shooter;
    protected OpenCV openCV;
    protected Battery battery;

    @Override
    protected Robot buildRobot() {
        Fishlo fishlo = new Fishlo(this);

        drive = (Drive) fishlo.getSubSystem("Drive");
        claw = (Claw) fishlo.getSubSystem("Claw");
        gyro = (Gyro) fishlo.getSubSystem("Gyro");
        intake = (Intake) fishlo.getSubSystem("Intake");
        shooter = (Shooter) fishlo.getSubSystem("Shooter");
        openCV = (OpenCV) fishlo.getSubSystem("OpenCV");
        battery = (Battery) fishlo.getSubSystem("Battery");

        return fishlo;
    }

    @Override
    public void main() {}

    @Override
    public void preMain() {}
}
