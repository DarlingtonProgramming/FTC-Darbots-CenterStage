package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team4100.Darvinci.DarvinciAutoBase;
import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;
import org.firstinspires.ftc.team4100.Darvinci.commands.DumpOnBackboard;
import org.firstinspires.ftc.team4100.Darvinci.commands.FollowTrajSequence;
import org.firstinspires.ftc.team4100.Darvinci.commands.SubsystemsToInitial;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciAutonomousSettings;

@Disabled
@Autonomous(name="blue")
public class NewBlueLeft extends DarvinciAutoBase {
    public static final Pose2d m_poseEstimate = new Pose2d(15.29, 62, Math.toRadians(90.00));

    public NewBlueLeft() {
        super(DarvinciCore.AllianceType.BLUE, m_poseEstimate);
    }

    @Override
    public void initAuto() {
        TrajectorySequence detectionTraj = null;

        switch (detectionResult) {
            case LEFT:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineTo(new Vector2d(23.42, 39.10))
                        .splineToLinearHeading(new Pose2d(53, 42.3, Math.toRadians(180.00)), Math.toRadians(-10.00))
                        .build();
                break;
            case MIDDLE:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineTo(new Vector2d(12.58, 34.45))
                        .splineToLinearHeading(new Pose2d(53, 35, Math.toRadians(180.00)), Math.toRadians(0.00))
                        .build();
                break;
            default:
                detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                        .lineToSplineHeading(new Pose2d(6.97, 34.84, Math.toRadians(46.00)))
                        .lineToSplineHeading(new Pose2d(53, 29.23, Math.toRadians(180.00)))
                        .build();
                break;
        }

        schedule(
                new SequentialCommandGroup(
                        new FollowTrajSequence(robot.m_drive, detectionTraj),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.m_slide.setRelPosition(DarvinciAutonomousSettings.SLIDE_DUMP_HEIGHT))
//                                new DriveToAprilTag(this, robot.m_drive, detectionTraj.end())
                        ),
                        new DumpOnBackboard(robot.m_bucket, robot.m_push, robot.m_slide),
                        new ParallelCommandGroup(
                                new FollowTrajSequence(robot.m_drive, robot.m_drive.trajectorySequenceBuilder(new Pose2d(getAprilTagPoseConstant(getDetectionId()), Math.toRadians(180)).plus(new Pose2d(DarvinciAutonomousSettings.APRILTAG_X_ROBOT_OFFSET, DarvinciAutonomousSettings.APRILTAG_Y_ROBOT_OFFSET)))
//                                        .splineToConstantHeading(new Vector2d(47.61, 54.97), Math.toRadians(44.60))
//                                        .splineToConstantHeading(new Vector2d(58.26, 60.39), Math.toRadians(0.00))
                                        .splineToConstantHeading(new Vector2d(46.84, 13.16), Math.toRadians(0.00))
                                        .splineToConstantHeading(new Vector2d(59, 13.16), Math.toRadians(0.00))
                                        .build()
                                ),
                                new SubsystemsToInitial(robot.m_bucket, robot.m_slide)
                        )
                )
        );
    }
}
