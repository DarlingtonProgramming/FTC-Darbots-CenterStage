package org.firstinspires.ftc.team4100.Darvinci;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciAutonomousSettings;
import org.firstinspires.ftc.team4100.FieldConstant;
import org.firstinspires.ftc.team4100.Darvinci.util.PoseStorage;
import org.firstinspires.ftc.team4100.Darvinci.vision.PropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public abstract class DarvinciAutoBase extends CommandOpMode {
    public enum Parking {
        LEFT, MIDDLE, RIGHT
    }

    private final DarvinciCore.AllianceType m_allianceType;
    private final Pose2d m_startingPose;
    protected PropDetectionProcessor.Detection detectionResult = PropDetectionProcessor.Detection.LEFT;
    protected DarvinciCore robot;
    protected WebcamName webcam;
    protected PropDetectionProcessor propDetectionProcessor;
    protected AprilTagProcessor aprilTagProcessor;
    protected VisionPortal vision;
    protected Pose2d m_aprilTagPose = null;

    public DarvinciAutoBase(DarvinciCore.AllianceType allianceType, Pose2d startPose) {
        m_allianceType = allianceType;
        m_startingPose = startPose;
    }

    public Pose2d getAprilTagPose(boolean relative) {
        if (!vision.getProcessorEnabled(aprilTagProcessor))
            vision.setProcessorEnabled(aprilTagProcessor, true);

        final int DETECTION_ID = getDetectionId();
        int attempts = 0;

        ArrayList<AprilTagPoseFtc> correctDetectionList = new ArrayList<>();

        while (true) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("AprilTags Detected", currentDetections.size());
            telemetry.update();

            for (AprilTagDetection detection : currentDetections) {
                // If it's the one we're looking for, add to the list
                if (detection.id == DETECTION_ID) {
                    correctDetectionList.add(detection.ftcPose);
                }
            }

            // Max number of attempts
            if (++attempts > DarvinciAutonomousSettings.APRILTAG_AVGED_ATTEMPTS) {
                break;
            }

            sleep(5);
        }

        if (correctDetectionList.size() == 0) {
            return null;
        }

        double x_avg = 0;
        double y_avg = 0;

        for (AprilTagPoseFtc pose : correctDetectionList) {
            x_avg += pose.x;
            y_avg += pose.y;
        }

        x_avg = x_avg / correctDetectionList.size();
        y_avg = y_avg / correctDetectionList.size();

        Pose2d aprilTagPose = new Pose2d(y_avg, x_avg);

        if (relative) {
            return aprilTagPose;
        }

        Pose2d currentPose = robot.m_drive.getPoseEstimate();
        currentPose.plus(aprilTagPose);

        return currentPose;
    }

    public static Vector2d getAprilTagPoseConstant(int num) {
        switch(num) {
            case 1:
                return FieldConstant.APRIL_TAG_ONE;
            case 2:
                return FieldConstant.APRIL_TAG_TWO;
            case 3:
                return FieldConstant.APRIL_TAG_THREE;
            case 4:
                return FieldConstant.APRIL_TAG_FOUR;
            case 5:
                return FieldConstant.APRIL_TAG_FIVE;
            case 6:
                return FieldConstant.APRIL_TAG_SIX;
        }
        return null;
    }

    public int getDetectionId() {
        switch(detectionResult) {
            case LEFT:
                return robot.ALLIANCE_TYPE == DarvinciCore.AllianceType.BLUE ? 1 : 4;
            case MIDDLE:
                return robot.ALLIANCE_TYPE == DarvinciCore.AllianceType.BLUE ? 2 : 5;
            default:
                return robot.ALLIANCE_TYPE == DarvinciCore.AllianceType.BLUE ? 3 : 6;
        }
    }

    @Override
    public void initialize() {
        robot = new DarvinciCore(hardwareMap, m_allianceType);
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        propDetectionProcessor = new PropDetectionProcessor(m_allianceType);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        vision = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .addProcessors(propDetectionProcessor, aprilTagProcessor)
                .build();
        robot.m_drive.setPoseEstimate(m_startingPose);

        // Vision portal should be prop detection first
        vision.setProcessorEnabled(aprilTagProcessor, false);
        vision.setProcessorEnabled(propDetectionProcessor, true);

        while (opModeInInit()) {
            detectionResult = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", detectionResult);
            telemetry.update();
            sleep(1);
        }

        vision.setProcessorEnabled(propDetectionProcessor, false);
        vision.setProcessorEnabled(aprilTagProcessor, true);

        initAuto();

        schedule(new InstantCommand(() -> {
            PoseStorage.currentPose = robot.m_drive.getPoseEstimate();
        }));
    }

    public abstract void initAuto();
}
