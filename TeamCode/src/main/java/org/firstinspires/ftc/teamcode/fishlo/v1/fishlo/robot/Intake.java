package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Intake extends SubSystem {

    private DcMotor intake;
    private DcMotor transfer;
    private Servo intakeLever;
    private CRServo transferWheels;
    private double INTAKE_HOME = 0.03;
    private double INTAKE_MAX = 0.065;

    public Intake(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        intake = robot.hardwareMap.dcMotor.get("intake");
        transfer = robot.hardwareMap.dcMotor.get("transfer");
        intakeLever = robot.hardwareMap.servo.get("intakeLever");
        transferWheels = robot.hardwareMap.crservo.get("transferWheels");
        intakeLever.setPosition(INTAKE_HOME);
    }

    @Override
    public void handle() {
        intakeRelease();
        intake.setPower(-robot.gamepad2.left_stick_y);
        transferWheels.setPower(robot.gamepad2.left_stick_y);
        transfer.setPower(-robot.gamepad2.left_stick_y);
    }

    public void intakeRelease() {
        intakeLever.setPosition(INTAKE_MAX);
    }

    public void stopIntake() {
        intake.setPower(0);
        transfer.setPower(0);
        intakeLever.setPosition(INTAKE_MAX);
        transferWheels.setPower(0);
    }

    public void startIntake() {
        intake.setPower(1);
        transfer.setPower(1);
        transferWheels.setPower(-1);
    }




    @Override
    public void stop() {
        stopIntake();
    }
}
