package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Intake extends SubSystem {

    private DcMotor intake;
    private DcMotor transfer;
    boolean intake_on = false;
    ElapsedTime intake_timer = new ElapsedTime();
    public static final double INTAKE_SPEED = -0.5;

    public Intake(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        intake = robot.hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void handle() {
        if (robot.gamepad2.y && !intake_on && intake_timer.seconds() > 3) {

            startIntake();
            intake_on = true;
            intake_timer.reset();
        }
        if (robot.gamepad2.y && intake_on && intake_timer.seconds() > 3) {
            stopIntake();
            intake_on = false;
            intake_timer.reset();
        }
    }

    public void startIntake() {
        intake.setPower(INTAKE_SPEED);
        transfer.setPower(0.5);
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
