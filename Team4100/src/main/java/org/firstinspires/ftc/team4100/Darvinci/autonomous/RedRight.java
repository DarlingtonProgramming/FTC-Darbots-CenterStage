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

public class RedRight extends DarvinciAutoBase {
    public static final Pose2d m_poseEstimate = DarvinciAutonomousSettings.RED_RIGHT_STARTING_POSE;
    protected final Parking m_parking;

    public RedRight(Parking parking) {
        super(DarvinciCore.AllianceType.RED, m_poseEstimate);
        m_parking = parking;
    }

    @Override
    public void initAuto() {
        TrajectorySequence detectionTraj = null;

        switch (detectionResult) {
            case LEFT:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineTo(new Vector2d(13.5, -58),
                                DarvinciChassis.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(20))
                        .lineToSplineHeading(new Pose2d(10.5, -33, Math.toRadians(345)),
                                DarvinciChassis.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(20))
                        .back(4,
                                DarvinciChassis.getVelocityConstraint(20, Math.toRadians(30), DriveConstants.TRACK_WIDTH),
                                DarvinciChassis.getAccelerationConstraint(20))
                        .lineToSplineHeading(new Pose2d(40, -30, Math.toRadians(180)))
                        .build();
                break;
            case MIDDLE:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineTo(new Vector2d(11.5, -34))
                        .forward(10)
                        .lineToSplineHeading(new Pose2d(40, -37, Math.toRadians(180)))
                        .build();
                break;
            default:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineToSplineHeading(new Pose2d(19.5, -40, Math.toRadians(268)))
                        .forward(14)
                        .lineToSplineHeading(new Pose2d(40, -42, Math.toRadians(180)))
                        .build();
                break;
        }

        schedule(
                new SequentialCommandGroup(
                        new FollowTrajSequence(robot.m_drive, detectionTraj),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.m_slide.setRelPosition(DarvinciAutonomousSettings.SLIDE_DUMP_HEIGHT)),
                                new DriveToAprilTag(this, robot.m_drive, detectionTraj.end())
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
                                                .lineToSplineHeading(new Pose2d(43, -11, Math.toRadians(180)))
                                                .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(180)))
                                                .build()
                                        ),
                                new SubsystemsToInitial(robot.m_bucket, robot.m_slide)
                        )
                )
        );
    }
}
