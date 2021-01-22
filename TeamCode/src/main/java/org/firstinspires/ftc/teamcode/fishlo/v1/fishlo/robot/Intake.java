package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Intake extends SubSystem {

    private DcMotor intake;
    private DcMotor transfer;
    private CRServo intakeLever;

    boolean intake_down = false;

    ElapsedTime intake_timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    int sum = 2;

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

        if (intake_down == false) {
            intakeDown();
            intake_down = true;
        }

        if (intake_down == true && robot.gamepad2.a) {
            intakeUp();
            intake_down = false;
        }
        else if (intake_down == false && robot.gamepad2.a) {
            intakeDown();
            intake_down = true;
        }

        intake.setPower(robot.gamepad2.left_stick_y);
        transfer.setPower(robot.gamepad2.left_stick_y);
    }

    public void stopIntake() {
        intake.setPower(0);
        transfer.setPower(0);
    }

    public void intakeUp() {
        intake_timer.reset();
        while (intake_timer.time() < 300) {
            intakeLever.setPower(1);
        }
        intakeLever.setPower(0);
    }

    public void intakeDown() {
        intake_timer.reset();
        while (intake_timer.time() < 1000 ) {
            intakeLever.setPower(-1);
        }
        intakeLever.setPower(0);
    }


    @Override
    public void stop() {
        stopIntake();
    }
}
