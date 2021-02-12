package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Claw extends SubSystem {
    private Servo claw;
    private DcMotor arm;

    public static final double CLAW_HOME = 0.39;
    public static final double CLAW_MAX = 0.88;
    public static double arm_speed = 0.5;
    ElapsedTime armTimer = new ElapsedTime();


    public Claw(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        claw = robot.hardwareMap.servo.get("claw");
        arm = robot.hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTimer.reset();
        close();
    }

    @Override
    public void handle() {
        arm_speed = robot.gamepad2.right_stick_x * 0.5;


        arm.setPower(0);
        arm.setPower(arm_speed);
        if (robot.gamepad2.x) {
            open();
        }
        else if (robot.gamepad2.b) {
            close();
        }

    }

    @Override
    public void stop() {
        open();
    }

    public void close() {
        claw.setPosition(CLAW_MAX);
    }

    public void open() {
        claw.setPosition(CLAW_HOME);
    }

    public void armDown() {
        armTimer.reset();
        while (armTimer.milliseconds() < 900) {
            arm.setPower(-0.5);
        }
        arm.setPower(0);

    }
    public void halfArmDown() {
        armTimer.reset();
        while (armTimer.milliseconds() < 600) {
            arm.setPower(-0.5);
        }
        arm.setPower(0);

    }

    public void halfArmUp() {
        armTimer.reset();
        while (armTimer.milliseconds() < 600) {
            arm.setPower(0.5);
        }
        arm.setPower(0);

    }

    public void armUp() {
        armTimer.reset();
        while (armTimer.milliseconds() < 900) {
            arm.setPower(0.5);
        }
        arm.setPower(0);

    }

    public void resetEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
