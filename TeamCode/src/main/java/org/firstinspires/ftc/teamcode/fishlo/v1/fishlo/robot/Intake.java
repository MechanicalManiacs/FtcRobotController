package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Intake extends SubSystem {

    private DcMotor intake;
    private DcMotor transfer;
    public static double INTAKE_SPEED = 0.5;
    public static double TRANSFER_SPEED = 0.25;

    int sum = 2;

    public Intake(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        intake = robot.hardwareMap.dcMotor.get("intake");
        transfer = robot.hardwareMap.dcMotor.get("transfer");
    }

    @Override
    public void handle() {
        if (robot.gamepad2.y) {
            sum ++;
            if (sum > 4) {
                sum = 2;
            }
        }

        if (robot.gamepad2.right_trigger > 0.5) {
            TRANSFER_SPEED = 0.75;
        }
        else {
            TRANSFER_SPEED = 0.25;
        }

        if (robot.gamepad2.left_trigger > 0.5) {
            INTAKE_SPEED = -1 * Math.abs(INTAKE_SPEED);
            TRANSFER_SPEED = -1 * Math.abs(TRANSFER_SPEED);
        }
        else {
            TRANSFER_SPEED = Math.abs(TRANSFER_SPEED);
            INTAKE_SPEED = Math.abs(INTAKE_SPEED);
        }

        int remainder = sum % 2;

        if (remainder == 1) {
            startIntake();
        }
        else {
            stopIntake();
        }
    }

    public void startIntake() {
        intake.setPower(INTAKE_SPEED);
        transfer.setPower(TRANSFER_SPEED);
    }

    public void stopIntake() {
        intake.setPower(0);
        transfer.setPower(0);
    }

    @Override
    public void stop() {
        stopIntake();
    }
}
