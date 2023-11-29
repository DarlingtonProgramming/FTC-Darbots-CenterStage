package org.firstinspires.ftc.team4100.Scrappy.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100.FieldConstant;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive.ScrappyChassis;

public class DriveToAprilTag extends CommandBase {
    private final ScrappyChassis m_drive;
    private final int m_aprilTagNumber;
    private double m_x_offset = 0;
    private double m_y_offset = 0;
    private boolean m_isFinished = false;

    public DriveToAprilTag(ScrappyChassis drive, int aprilTagNumber) {
        m_drive = drive;
        m_aprilTagNumber = aprilTagNumber;
    }

    public DriveToAprilTag(ScrappyChassis drive, int aprilTagNumber, double x_offset) {
        m_drive = drive;
        m_aprilTagNumber = aprilTagNumber;
        m_x_offset = x_offset;
    }

    public DriveToAprilTag(ScrappyChassis drive, int aprilTagNumber, double x_offset, double y_offset) {
        m_drive = drive;
        m_aprilTagNumber = aprilTagNumber;
        m_x_offset = x_offset;
        m_y_offset = y_offset;
    }


    @Override
    public void execute() {
        if (!m_isFinished && !m_drive.isBusy()) {
            Pose2d currentPose = m_drive.getPoseEstimate();
            Vector2d aprilTagPose = FieldConstant.getAprilTagPose(m_aprilTagNumber);

            m_drive.followTrajectorySequenceAsync(
                    m_drive.trajectorySequenceBuilder(currentPose)
                            .lineTo(aprilTagPose.plus(new Vector2d(m_x_offset, m_y_offset)))
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
        Pose2d currentPose = m_drive.getPoseEstimate();
        m_drive.setPoseEstimate(new Pose2d(FieldConstant.getAprilTagPose(m_aprilTagNumber), currentPose.getHeading()).plus(new Pose2d(m_x_offset, m_y_offset)));
    }
}
