package org.firstinspires.ftc.team4100.Scrappy;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team4100.Scrappy.util.PoseStorage;
import org.firstinspires.ftc.team4100.Scrappy.vision.PropDetectionProcessor;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore.AllianceType;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore.AllianceSide;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class ScrappyAutoBase extends CommandOpMode {
    public enum Parking {
        LEFT, MIDDLE, RIGHT
    }

    protected final AllianceType m_allianceType;
    protected final AllianceSide m_allianceSide;
    protected final Pose2d m_startingPose;
    protected PropDetectionProcessor.Detection detectionResult = PropDetectionProcessor.Detection.LEFT;
    protected ScrappyCore robot;
    protected WebcamName webcam;

    protected PropDetectionProcessor propDetectionProcessor;
    protected VisionPortal vision;

    public ScrappyAutoBase(AllianceType allianceType, AllianceSide allianceSide, Pose2d startPose) {
        m_allianceType = allianceType;
        m_allianceSide = allianceSide;
        m_startingPose = startPose;
    }

    @Override
    public void initialize() {
        robot = new ScrappyCore(hardwareMap, m_allianceType, m_allianceSide);
        robot.m_drive.setPoseEstimate(m_startingPose);
        robot.m_lift.toInitial();
        robot.m_dropper.back();

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        propDetectionProcessor = new PropDetectionProcessor(m_allianceType, m_allianceSide);
        vision = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .addProcessor(propDetectionProcessor)
                .build();

        while (opModeInInit()) {
            detectionResult = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", detectionResult);
            telemetry.update();
            sleep(1);
        }

        initAuto();
    }

    @Override
    public void run() {
        super.run();
        robot.m_drive.update();
        PoseStorage.currentPose = robot.m_drive.getPoseEstimate();
    }

    public abstract void initAuto();
}
