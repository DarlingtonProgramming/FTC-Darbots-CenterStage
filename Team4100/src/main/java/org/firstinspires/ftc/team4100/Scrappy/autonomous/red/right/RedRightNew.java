package org.firstinspires.ftc.team4100.Scrappy.autonomous.red.right;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team4100.Scrappy.ScrappyAutoBase;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;
import org.firstinspires.ftc.team4100.Scrappy.commands.CommonAutoCommand;
import org.firstinspires.ftc.team4100.Scrappy.commands.FollowTrajSequence;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.team4100.Scrappy.vision.PropDetectionProcessor;

@Autonomous
public class RedRightNew extends ScrappyAutoBase {

    public static final Pose2d m_poseEstimate = new Pose2d(14, -61.75, Math.toRadians(90));
    private final Parking m_parking;

    public RedRightNew() {
        super(ScrappyCore.AllianceType.RED, ScrappyCore.AllianceSide.RIGHT, m_poseEstimate);
        m_parking = Parking.MIDDLE;
    }

    public RedRightNew(Parking parking) {
        super(ScrappyCore.AllianceType.RED, ScrappyCore.AllianceSide.RIGHT, m_poseEstimate);
        m_parking = parking;
    }

    @Override
    public void initAuto() {
        // Detection
        TrajectorySequenceBuilder detectionTrajBuilder = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate);

        switch (detectionResult) {
            case LEFT:
                detectionTrajBuilder
                        .lineToSplineHeading(new Pose2d(12, -31, Math.toRadians(0)))
                        .back(1);
                break;
            case MIDDLE:
                detectionTrajBuilder.lineToSplineHeading(new Pose2d(13, -35, Math.toRadians(270)));
                break;
            case RIGHT:
                detectionTrajBuilder.lineToSplineHeading(new Pose2d(21, -42, Math.toRadians(270)));
                break;
        }

        TrajectorySequence detectionTraj = detectionTrajBuilder.build();

        // Backboard
        TrajectorySequenceBuilder backboardTrajBuilder = robot.m_drive.trajectorySequenceBuilder(detectionTraj.end())
                .addDisplacementMarker(1.2, () -> {
                    robot.m_lift.toInitial();
                    robot.m_dropper.back();
                    robot.m_conveyor.up();
                });

        if (detectionResult != PropDetectionProcessor.Detection.LEFT) {
            backboardTrajBuilder.lineToSplineHeading(new Pose2d(27, detectionResult == PropDetectionProcessor.Detection.RIGHT ? -48.5 : -41, Math.toRadians(270)));
        }

        double backboardTrajY = detectionResult == PropDetectionProcessor.Detection.RIGHT ? -45.5 : detectionResult == PropDetectionProcessor.Detection.MIDDLE ? -37 : -28.5;
        backboardTrajBuilder.lineToSplineHeading(new Pose2d(52, backboardTrajY, Math.toRadians(180)));

        TrajectorySequence backboardTraj = backboardTrajBuilder.build();

        TrajectorySequence stackTraj = robot.m_drive.trajectorySequenceBuilder(backboardTraj.end())
                .addDisplacementMarker(2.3, () -> robot.m_lift.toInitial())
                .lineToSplineHeading(new Pose2d(34, -11, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-54, -14, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-57, -18, Math.toRadians(180)))
                .build();

        TrajectorySequence asd = robot.m_drive.trajectorySequenceBuilder(stackTraj.end())
                .addDisplacementMarker(10, () -> robot.m_intake.stop())
                .lineToSplineHeading(new Pose2d(34, -10.5, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(51.5, -37, Math.toRadians(180)))
                .build();

        schedule(new SequentialCommandGroup(
                new CommonAutoCommand(robot, detectionTraj, backboardTraj),
                new FollowTrajSequence(robot.m_drive, robot.m_drive.trajectorySequenceBuilder(backboardTraj.end())
                        .lineTo(new Vector2d(47, -54))
                        .lineTo(new Vector2d(57, -54))
                        .build()
                )
        ));
    }
}
