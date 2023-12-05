package org.firstinspires.ftc.team4100.Scrappy.autonomous.red.left;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team4100.Scrappy.ScrappyAutoBase;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;
import org.firstinspires.ftc.team4100.Scrappy.commands.CommonAutoCommand;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive.ScrappyChassis;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.team4100.Scrappy.vision.PropDetectionProcessor;

@Autonomous
@Config
public class RedLeftStackk extends ScrappyAutoBase {
    public static double delay = 0;
    public static final Pose2d m_poseEstimate = new Pose2d(-38, -61.75, Math.toRadians(90));
    private final Parking m_parking;

    public RedLeftStackk() {
        super(ScrappyCore.AllianceType.RED, ScrappyCore.AllianceSide.LEFT, m_poseEstimate);
        m_parking = Parking.MIDDLE;
    }

    public RedLeftStackk(Parking parking) {
        super(ScrappyCore.AllianceType.RED, ScrappyCore.AllianceSide.LEFT, m_poseEstimate);
        m_parking = parking;
    }
    @Override
    public void initAuto() {
        // Detection
        TrajectorySequenceBuilder detectionTrajBuilder = robot.m_drive.trajectorySequenceBuilder(m_poseEstimate);

        switch (detectionResult) {
            case LEFT:
                detectionTrajBuilder.lineTo(new Vector2d(-46, -16));
                break;
            case MIDDLE:
                detectionTrajBuilder.lineTo(new Vector2d(-37, -15));
                break;
            case RIGHT:
                detectionTrajBuilder
                        .lineToSplineHeading(new Pose2d(-36.5, -31, (Math.toRadians(180))))
                        .back(2);
                break;
        }

        TrajectorySequence detectionTraj = detectionTrajBuilder.build();

        // Backboard
        double backboardTrajY = detectionResult == PropDetectionProcessor.Detection.RIGHT ? -46.5 : detectionResult == PropDetectionProcessor.Detection.MIDDLE ? -37 : -33;

        TrajectorySequence backboardTraj = robot.m_drive.trajectorySequenceBuilder(detectionTraj.end())
                .lineToSplineHeading(new Pose2d(-37, -10.7, Math.toRadians(180)), ScrappyChassis.getVelocityConstraint(65, Math.toRadians(90), DriveConstants.TRACK_WIDTH), ScrappyChassis.getAccelerationConstraint(65))
                .lineTo(new Vector2d(34, -11))
                .waitSeconds(delay)
                .lineToSplineHeading(new Pose2d(50.2, backboardTrajY, Math.toRadians(180)))
                .addDisplacementMarker(1.5, () -> {
                    robot.m_lift.toInitial();
                    robot.m_dropper.back();
                    robot.m_conveyor.up();
                })
                .build();

        schedule(new CommonAutoCommand(robot, detectionTraj, backboardTraj));
    }
}
