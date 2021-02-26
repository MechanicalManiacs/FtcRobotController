package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Battery extends SubSystem {

    public Battery(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }

    public double getVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : robot.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
