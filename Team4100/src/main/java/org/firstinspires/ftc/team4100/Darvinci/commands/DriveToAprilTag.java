package org.firstinspires.ftc.team4100.Darvinci.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100.Darvinci.DarvinciAutoBase;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;

public class DriveToAprilTag extends CommandBase {
    private final DarvinciAutoBase m_base;
    private final DarvinciChassis m_drive;
    private final Pose2d m_lastPose;
    private final double m_x_off;
    private final double m_y_off;
    private Pose2d m_aprilTagPose = null;
    private boolean m_isFinished = false;

    public DriveToAprilTag(DarvinciAutoBase base, DarvinciChassis drive, Pose2d lastPose, double x_off, double y_off) {
        m_base = base;
        m_drive = drive;
        m_lastPose = lastPose;
        m_x_off = x_off;
        m_y_off = y_off;
    }

    @Override
    public void execute() {
        if (!m_isFinished && !m_drive.isBusy()) {
            m_aprilTagPose = m_base.getAprilTagPose(true);
            Pose2d currentPose = m_drive.getPoseEstimate();
            m_drive.followTrajectorySequenceAsync(
                    m_drive.trajectorySequenceBuilder(m_lastPose)
                            .lineToLinearHeading(new Pose2d(currentPose.getX() + (m_aprilTagPose.getX() + m_x_off), currentPose.getY() + (m_aprilTagPose.getY() + m_y_off), Math.toRadians(180)))
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
        m_drive.setPoseEstimate(new Pose2d(DarvinciAutoBase.getAprilTagPoseConstant(m_base.getDetectionId()), Math.toRadians(180)).plus(new Pose2d(m_x_off, m_y_off)));
    }
}
