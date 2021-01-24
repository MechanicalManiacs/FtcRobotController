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

    private UGRectDetector detector = new UGRectDetector(robot.hardwareMap, "Webcam 1");

    private UGRectDetector.Stack stack;

    private final int RECT_WIDTH = 50;
    private final int RECT_HEIGHT = 50;

    public enum targetZone {
        A,
        B,
        C,
        X
    }

    @Override
    public void init() {}

    @Override
    public void handle() {}

    @Override
    public void stop() {}

    public void initVision() {
        detector.init();
        detector.setRectangleSize(RECT_WIDTH, RECT_HEIGHT);
    }

    public targetZone getTargetZone() {
        stack = detector.getStack();

        targetZone targetZone = OpenCV.targetZone.X;

        switch (stack) {
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

    public UGRectDetector.Stack getHeight() {
        stack = detector.getStack();

        return stack;
    }

}
