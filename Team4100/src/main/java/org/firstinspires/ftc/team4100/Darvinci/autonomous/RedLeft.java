package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.team4100.Darvinci.DarvinciAutoBase;
import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;
import org.firstinspires.ftc.team4100.Darvinci.commands.DriveToAprilTag;
import org.firstinspires.ftc.team4100.Darvinci.commands.DumpOnBackboard;
import org.firstinspires.ftc.team4100.Darvinci.commands.FollowTrajSequence;
import org.firstinspires.ftc.team4100.Darvinci.commands.SubsystemsToInitial;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciAutonomousSettings;

public class RedLeft extends DarvinciAutoBase {
    public static final Pose2d m_poseEstimate = DarvinciAutonomousSettings.RED_LEFT_STARTING_POSE;
    protected final Parking m_parking;

    public RedLeft(Parking parking) {
        super(DarvinciCore.AllianceType.RED, m_poseEstimate);
        m_parking = parking;
    }

    @Override
    public void initAuto() {
        TrajectorySequence detectionTraj = null;

        switch (detectionResult) {
            case LEFT:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineTo(new Vector2d(-46, -40))
                        .lineToSplineHeading(new Pose2d(-35.5, -58.2, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(18, -58.2, Math.toRadians(180)))
                        .waitSeconds(4)
                        .lineToSplineHeading(new Pose2d(44, -29.5, Math.toRadians(180)))
                        .build();
                break;
            case MIDDLE:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineTo(new Vector2d(-36.5, -34))
                        .lineToSplineHeading(new Pose2d(-39, -58.2, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(18, -58.2, Math.toRadians(180)))
                        .waitSeconds(4)
                        .lineToSplineHeading(new Pose2d(45, -37, Math.toRadians(180)))
                        .build();
                break;
            default:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineTo(new Vector2d(-39.5, -58),
                                DarvinciChassis.getVelocityConstraint(13, Math.toRadians(26), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(13))
                        .lineToSplineHeading(new Pose2d(-36.5, -33, Math.toRadians(210)),
                                DarvinciChassis.getVelocityConstraint(13, Math.toRadians(23), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(13))
                        .back(5,
                                DarvinciChassis.getVelocityConstraint(13, Math.toRadians(26), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(13))
                        .lineToSplineHeading(new Pose2d(-41.5, -36.5, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(-39, -58.2, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(18, -58.2, Math.toRadians(180)))
                        .waitSeconds(2.5)
                        .lineToSplineHeading(new Pose2d(44, -43, Math.toRadians(180)))
                        .build();
                break;
        }

        schedule(
                new SequentialCommandGroup(
                        new FollowTrajSequence(robot.m_drive, detectionTraj),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.m_slide.setRelPosition(DarvinciAutonomousSettings.SLIDE_DUMP_HEIGHT)),
                                new DriveToAprilTag(this, robot.m_drive, detectionTraj.end(), getDetectionId() == 6 ? -0.4 : -0.78, getDetectionId() == 4 ? -1.8 : getDetectionId() == 6 ? -1 : 1.7)
                        ),
                        new DumpOnBackboard(robot.m_bucket, robot.m_push, robot.m_slide),
                        new ParallelCommandGroup(
                                m_parking == Parking.RIGHT ?
                                        new FollowTrajSequence(robot.m_drive, robot.m_drive.trajectorySequenceBuilder(new Pose2d(getAprilTagPoseConstant(getDetectionId()), Math.toRadians(180)).plus(new Pose2d(DarvinciAutonomousSettings.APRILTAG_X_ROBOT_OFFSET, DarvinciAutonomousSettings.APRILTAG_Y_ROBOT_OFFSET)))
                                                .forward(5)
                                                .lineToSplineHeading(new Pose2d(41, -59, Math.toRadians(180)))
                                                .lineToSplineHeading(new Pose2d(57, -59, Math.toRadians(180)))
                                                .build()
                                        ) :
                                        new FollowTrajSequence(robot.m_drive, robot.m_drive.trajectorySequenceBuilder(new Pose2d(getAprilTagPoseConstant(getDetectionId()), Math.toRadians(180)).plus(new Pose2d(DarvinciAutonomousSettings.APRILTAG_X_ROBOT_OFFSET, DarvinciAutonomousSettings.APRILTAG_Y_ROBOT_OFFSET)))
                                                .forward(5)
                                                .lineToSplineHeading(new Pose2d(43, -13, Math.toRadians(180)))
                                                .lineToSplineHeading(new Pose2d(58.5, -13, Math.toRadians(180)))
                                                .build()
                                        ),
                                new SubsystemsToInitial(robot.m_bucket, robot.m_slide)
                        )
                )
        );
    }
}
