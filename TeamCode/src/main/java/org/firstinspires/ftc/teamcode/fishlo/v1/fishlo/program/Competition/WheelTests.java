package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class WheelTests extends FishloAutonomousProgram {


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
            if (gamepad1.a) {
                drive.testFrontRight();
                telemetry.addData("Status", "Testing frontRight");
                telemetry.update();
            }
            else if (gamepad1.b) {
                drive.testFrontLeft();
                telemetry.addData("Status", "Testing frontLeft");
                telemetry.update();
            }
            else if (gamepad1.x) {
                drive.testBackLeft();
                telemetry.addData("Status", "Testing backLeft");
                telemetry.update();
            }
            else if (gamepad1.y) {
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
