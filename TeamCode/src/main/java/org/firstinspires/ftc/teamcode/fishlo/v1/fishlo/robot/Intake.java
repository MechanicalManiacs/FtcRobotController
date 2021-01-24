package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Intake extends SubSystem {

    private DcMotor intake;
    private DcMotor transfer;
    private CRServo intakeLever;

    public Intake(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        intake = robot.hardwareMap.dcMotor.get("intake");
        transfer = robot.hardwareMap.dcMotor.get("transfer");
        intakeLever = robot.hardwareMap.crservo.get("intakeLever");
    }

    @Override
    public void handle() {
        intake.setPower(robot.gamepad2.left_stick_y);
        transfer.setPower(robot.gamepad2.left_stick_y);
        intakeLever.setPower((robot.gamepad2.left_trigger-robot.gamepad2.right_trigger)/2);
    }

    public void stopIntake() {
        intake.setPower(0);
        transfer.setPower(0);
        intakeLever.setPower(0);
    }


    @Override
    public void stop() {
        stopIntake();
    }
}
