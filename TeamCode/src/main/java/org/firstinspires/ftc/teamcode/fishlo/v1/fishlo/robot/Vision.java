package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import com.arcrobotics.ftclib.vision.InternalCameraExample;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

public class Vision extends SubSystem {

    public Vision(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {}

    @Override
    public void handle() {}

    @Override
    public void stop() {}

    private static final int CAMERA_WIDTH = 320;
    private static final int CAMERA_HEIGHT = 240;

    private static final int HORIZON = 130;

    private static final boolean DEBUG = false;

    private static final boolean USING_WEBCAM = true;
    private static final String WEBCAM_NAME = "Webcam 1";

    public enum targetZone {
        A,
        B,
        C,
        X
    }

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    public void initVision() {
        int cameraMonitorViewId = this.robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        }
        else {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(robot.telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);
        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    UGContourRingPipeline.Height height;

    public targetZone getTargetZone() {

        height = pipeline.getHeight();
        targetZone targetZone = Vision.targetZone.X;

        if (height == height.ONE) {
            targetZone = Vision.targetZone.B;
        }
        else if (height == height.FOUR) {
            targetZone = Vision.targetZone.C;
        }
        else if (height == height.ZERO) {
            targetZone = Vision.targetZone.A;
        }

        return targetZone;
    }

    public UGContourRingPipeline.Height getHeight() {
        return height;
    }
}
