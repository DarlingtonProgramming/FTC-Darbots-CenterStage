package org.firstinspires.ftc.team4100.Scrappy.autonomous.blue.left;

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
public class BlueLeftNew extends ScrappyAutoBase {

    public static final Pose2d m_poseEstimate = new Pose2d(14, 61.75, Math.toRadians(270.00));
    private final Parking m_parking;

    public BlueLeftNew() {
        super(ScrappyCore.AllianceType.BLUE, ScrappyCore.AllianceSide.LEFT, m_poseEstimate);
        m_parking = Parking.MIDDLE;
    }

    public BlueLeftNew(Parking parking) {
        super(ScrappyCore.AllianceType.BLUE, ScrappyCore.AllianceSide.LEFT, m_poseEstimate);
        m_parking = parking;
    }

    @Override
    public void initAuto() {
        // Detection
        TrajectorySequenceBuilder detectionTrajBuilder = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate);

        switch (detectionResult) {
            case LEFT:
                detectionTrajBuilder.lineToSplineHeading(new Pose2d(24, 42, Math.toRadians(90)));
                break;
            case MIDDLE:
                detectionTrajBuilder.lineToSplineHeading(new Pose2d(13, 35, (Math.toRadians(90) + 1e-6)));
                break;
            case RIGHT:
                detectionTrajBuilder
                        .lineToSplineHeading(new Pose2d(12, 31, Math.toRadians(0)))
                        .back(1);
                break;
        }

        TrajectorySequence detectionTraj = detectionTrajBuilder.build();

        // Backboard
        TrajectorySequenceBuilder backboardTrajBuilder = robot.m_drive.trajectorySequenceBuilder(detectionTraj.end())
                .addDisplacementMarker(() -> {
                    robot.m_lift.toInitial();
                    robot.m_dropper.back();
                    robot.m_conveyor.up();
                });

        if (detectionResult != PropDetectionProcessor.Detection.RIGHT) {
            backboardTrajBuilder.lineToSplineHeading(new Pose2d(27, detectionResult == PropDetectionProcessor.Detection.LEFT ? 47 : 41, Math.toRadians(90)));
        }

        double backboardTrajY = detectionResult == PropDetectionProcessor.Detection.LEFT ? 44 : detectionResult == PropDetectionProcessor.Detection.MIDDLE ? 35.5 : 28.5;
        backboardTrajBuilder.lineToSplineHeading(new Pose2d(51.5, backboardTrajY, Math.toRadians(180)));

        TrajectorySequence backboardTraj = backboardTrajBuilder.build();

        schedule(new SequentialCommandGroup(
                new CommonAutoCommand(robot, detectionTraj, backboardTraj),
                new FollowTrajSequence(robot.m_drive, robot.m_drive.trajectorySequenceBuilder(backboardTraj.end())
                        .lineTo(new Vector2d(47, 10.5))
                        .lineTo(new Vector2d(57, 11.5))
                        .build()
                )
        ));
    }
}
