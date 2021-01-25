package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGRectDetector;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.opencv.core.Scalar;

public class OpenCV extends SubSystem {

    public OpenCV(Robot robot) {
        super(robot);
    }

    private UGContourRingDetector detector = new UGContourRingDetector(robot.hardwareMap, "Webcam 1", robot.telemetry, true);
    private UGContourRingPipeline.Height height;

    private final int HORIZON = 150;

    public enum targetZone {
        A,
        B,
        C,
        X
    }

    public void initVision() {
        detector.init();
        UGContourRingPipeline.Config.setCAMERA_WIDTH(320);
        UGContourRingPipeline.Config.setHORIZON(HORIZON);
    }

    @Override
    public void init() {}

    @Override
    public void handle() {}

    @Override
    public void stop() {detector.camera.stopStreaming();}

    public targetZone getTargetZone() {
        height = detector.getHeight();

        targetZone targetZone;

        if (height == UGContourRingPipeline.Height.ZERO) {
            targetZone = OpenCV.targetZone.A;
        }
        else if (height == UGContourRingPipeline.Height.ONE) {
            targetZone = OpenCV.targetZone.B;
        }
        else if (height == UGContourRingPipeline.Height.FOUR) {
            targetZone = OpenCV.targetZone.C;
        }
        else {
            targetZone = OpenCV.targetZone.X;
        }
        return targetZone;
    }

    public UGContourRingPipeline.Height getHeight() {
        height = detector.getHeight();

        return height;
    }

    public void stopAll() {
        detector.camera.closeCameraDevice();
    }
}
