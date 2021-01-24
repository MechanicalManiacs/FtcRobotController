package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class OpenCV extends SubSystem {

    public OpenCV(Robot robot) {
        super(robot);
    }

    private UGContourRingDetector detector = new UGContourRingDetector(robot.hardwareMap, "Webcam 1");
    private UGContourRingPipeline.Height height;

    private final int HORIZON = 100;

    public enum targetZone {
        A,
        B,
        C,
        X
    }

    public void initVision() {
        detector.init();
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

        targetZone targetZone = OpenCV.targetZone.X;

        switch (height) {
            case ZERO:
                targetZone = targetZone.A;
                break;
            case ONE:
                targetZone = targetZone.B;
                break;
            case FOUR:
                targetZone = targetZone.C;
                break;
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
