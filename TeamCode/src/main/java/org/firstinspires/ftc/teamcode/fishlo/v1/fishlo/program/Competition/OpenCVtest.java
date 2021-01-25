package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class OpenCVtest extends LinearOpMode {

    private final int HORIZON = 200;

    public enum targetZone {
        A,
        B,
        C,
        X
    }


    @Override
    public void runOpMode() throws InterruptedException {

        UGContourRingDetector detector = new UGContourRingDetector(hardwareMap, "Webcam 1", telemetry, true);
        detector.init();
        UGContourRingPipeline.Config.setCAMERA_WIDTH(320);
        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        while (isStarted()) {

            UGContourRingPipeline.Height height = detector.getHeight();

        }

        waitForStart();

        telemetry.addLine("Started");
    }
}
