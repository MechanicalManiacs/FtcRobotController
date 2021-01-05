package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class WheelTests extends FishloAutonomousProgram {

    boolean a = gamepad1.a;
    boolean b = gamepad1.b;
    boolean x = gamepad1.x;
    boolean y = gamepad1.y;

    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void preMain() {
        telemetry.addLine("Make sure to turn off 30 second timer!");
        telemetry.update();
    }

    @Override
    public void main() {
        while (opModeIsActive()) {
            if (a) {
                drive.testFrontRight();
                telemetry.addData("Status", "Testing frontRight");
                telemetry.update();
            }
            else if (b) {
                drive.testFrontLeft();
                telemetry.addData("Status", "Testing frontLeft");
                telemetry.update();
            }
            else if (x) {
                drive.testBackLeft();
                telemetry.addData("Status", "Testing backLeft");
                telemetry.update();
            }
            else if (y) {
                drive.testBackRight();
                telemetry.addData("Status", "Testing backRight");
                telemetry.update();
            }
            else {
                drive.stop();
                telemetry.addData("Status", "Stopped");
                telemetry.update();
            }
            //test
        }
    }
}
