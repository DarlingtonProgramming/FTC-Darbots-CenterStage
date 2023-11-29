package org.firstinspires.ftc.team4100.Scrappy.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.team4100.Scrappy.commands.FollowTrajSequence;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive.ScrappyChassis;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyAutoBase;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;

public class BlueLeft extends ScrappyAutoBase {
    public static final Pose2d m_poseEstimate = new Pose2d(14, 61.75, Math.toRadians(270.00));
    protected final Parking m_parking;

    public BlueLeft(Parking parking) {
        super(ScrappyCore.AllianceType.BLUE, m_poseEstimate);
        m_parking = parking;
    }

    @Override
    public void initAuto() {
        TrajectorySequence detectionTraj = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                // middle
//                .lineTo(new Vector2d(12.58, 34))
//                .lineToSplineHeading(new Pose2d(52.5, 35.05, Math.toRadians(180)))

                // left
//                .lineTo(new Vector2d(21.2, 36))
//                .lineToSplineHeading(new Pose2d(52.5, 41, Math.toRadians(180)))

                // right
                .lineTo(new Vector2d(11.5, 42))
                .lineToSplineHeading(new Pose2d(23, 43, Math.toRadians(180)), ScrappyChassis.getVelocityConstraint(20, Math.toRadians(60), DriveConstants.TRACK_WIDTH), ScrappyChassis.getAccelerationConstraint(20))
                .lineTo(new Vector2d(23, 31), ScrappyChassis.getVelocityConstraint(20, Math.toRadians(60), DriveConstants.TRACK_WIDTH), ScrappyChassis.getAccelerationConstraint(20))
                .forward(13.3, ScrappyChassis.getVelocityConstraint(20, Math.toRadians(60), DriveConstants.TRACK_WIDTH), ScrappyChassis.getAccelerationConstraint(20))
                .lineTo(new Vector2d(52.5, 28.5))
                .build();

        schedule(
                new SequentialCommandGroup(
                        new FollowTrajSequence(robot.m_drive, detectionTraj),
                        new InstantCommand(() -> robot.m_lift.setRelativePosition(1250)),
                        new WaitCommand(1000),
                        new InstantCommand(() -> robot.m_drive.setMotorPowers(-0.2, -0.2, -0.2, -0.2)),
                        new WaitCommand(950),
                        new InstantCommand(robot.m_dropper::drop),
                        new InstantCommand(() -> robot.m_drive.setMotorPowers(0, 0, 0, -0)),
                        new WaitCommand(1000),
                        new FollowTrajSequence(robot.m_drive, robot.m_drive.trajectorySequenceBuilder(detectionTraj.end())
                                .addDisplacementMarker(2.3, () -> robot.m_lift.toInitial())
                                .forward(3)
                                .build())
                )
        );
    }
}
