package org.firstinspires.ftc.team4100.Darvinci.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100.Darvinci.DarvinciAutoBase;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciAutonomousSettings;

public class DriveToAprilTag extends CommandBase {
    private final DarvinciAutoBase m_base;
    private final DarvinciChassis m_drive;
    private final Pose2d m_lastPose;
    private Pose2d m_aprilTagPose = null;
    private boolean m_isFinished = false;

    private double m_xoffset = 0;
    private double m_yoffset = 0;

    public DriveToAprilTag(DarvinciAutoBase base, DarvinciChassis drive, Pose2d lastPose) {
        m_base = base;
        m_drive = drive;
        m_lastPose = lastPose;
    }

    public DriveToAprilTag(DarvinciAutoBase base, DarvinciChassis drive, Pose2d lastPose, double x_offset, double y_offset) {
        m_base = base;
        m_drive = drive;
        m_lastPose = lastPose;
        m_xoffset = x_offset;
        m_yoffset = y_offset;
    }

    @Override
    public void execute() {
        if (!m_isFinished && !m_drive.isBusy()) {
            m_aprilTagPose = m_base.getAprilTagPose(true);
            Pose2d currentPose = m_drive.getPoseEstimate();
            m_drive.followTrajectorySequenceAsync(
                    m_drive.trajectorySequenceBuilder(m_lastPose)
                            .lineToLinearHeading(new Pose2d(currentPose.getX() + (m_aprilTagPose.getX() + DarvinciAutonomousSettings.APRILTAG_X_OFFSET + m_xoffset), currentPose.getY() + (m_aprilTagPose.getY() + DarvinciAutonomousSettings.APRILTAG_Y_OFFSET + m_yoffset), Math.toRadians(180)))
                            .build()
            );
            m_isFinished = true;
        }
        m_drive.update();
    }

    @Override
    public boolean isFinished() {
        return (m_isFinished && !m_drive.isBusy());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setPoseEstimate(new Pose2d(DarvinciAutoBase.getAprilTagPoseConstant(m_base.getDetectionId()), Math.toRadians(180)).plus(new Pose2d(DarvinciAutonomousSettings.APRILTAG_X_ROBOT_OFFSET, DarvinciAutonomousSettings.APRILTAG_Y_ROBOT_OFFSET)));
    }
}
