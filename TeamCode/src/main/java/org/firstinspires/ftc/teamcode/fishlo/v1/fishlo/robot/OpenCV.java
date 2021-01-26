package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
// com/arcrobotics/ftclib/vision/UGContourRingDetector.kt

public class OpenCV extends SubSystem {

    public OpenCV(Robot robot) {
        super(robot);
    }

    private UGContourRingPipeline.Height height;

    public enum targetZone {
        A,
        B,
        C,
        X
    }
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    public void initVision() {
        int cameraMonitorViewId = robot
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        robot.hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(robot.hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(robot.telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    public targetZone getTargetZone() {
        height = pipeline.getHeight();

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
        return height;
    }


    @Override
    public void init() {}

    @Override
    public void handle() {}

    @Override
    public void stop() {camera.stopStreaming();}


}
