package org.firstinspires.ftc.team4100.Scrappy.autonomous.blue.left;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team4100.Scrappy.ScrappyAutoBase;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;
import org.firstinspires.ftc.team4100.Scrappy.commands.FollowTrajSequence;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.team4100.Scrappy.vision.PropDetectionProcessor;

@Autonomous
public class BlueLeftStackk extends ScrappyAutoBase {

    public static final Pose2d m_poseEstimate = new Pose2d(14, 61.75, Math.toRadians(270.00));

    public BlueLeftStackk() {
        super(ScrappyCore.AllianceType.BLUE, ScrappyCore.AllianceSide.LEFT, m_poseEstimate);
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
            default:
                detectionTrajBuilder
                        .lineToSplineHeading(new Pose2d(12, 31, Math.toRadians(0)))
                        .back(1);
                break;
        }

        detectionTrajBuilder.addDisplacementMarker(() -> robot.m_lift.setRelativePosition(400));

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

        double backboardTrajY = detectionResult == PropDetectionProcessor.Detection.LEFT ? 44 : detectionResult == PropDetectionProcessor.Detection.MIDDLE ? 35.5 : 30;
        backboardTrajBuilder.lineToSplineHeading(new Pose2d(51.5, backboardTrajY, Math.toRadians(180)));

        TrajectorySequence backboardTraj = backboardTrajBuilder.build();

        TrajectorySequence stackTraj = robot.m_drive.trajectorySequenceBuilder(backboardTraj.end())
                .addDisplacementMarker(2.3, () -> robot.m_lift.toInitial())
                .lineToSplineHeading(new Pose2d(34, 11, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-54, 14, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-57, 18, Math.toRadians(180)))
                .build();

        TrajectorySequence asd = robot.m_drive.trajectorySequenceBuilder(stackTraj.end())
                .addDisplacementMarker(10, () -> robot.m_intake.stop())
                .lineToSplineHeading(new Pose2d(34, 10.5, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(51.5, 32.5, Math.toRadians(180)))
                .build();

        schedule(
                new SequentialCommandGroup(
                    new FollowTrajSequence(robot.m_drive, detectionTraj),
                    new InstantCommand(robot.m_dropper::drop),
                    new WaitCommand(400),
                    new InstantCommand(() -> {
                        robot.m_lift.setPower(0.4);
                        robot.m_lift.setRelativePosition(125);
                    }),
                    new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(525)),
                    new InstantCommand(() -> robot.m_lift.setPower(1)),
                    new FollowTrajSequence(robot.m_drive, backboardTraj),
                    new InstantCommand(robot.m_conveyor::stop),
                    new InstantCommand(() -> robot.m_lift.setRelativePosition(1050)),
                    new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(1050)),
                    new InstantCommand(() -> robot.m_drive.setMotorPowers(-0.35, -0.35, -0.35, -0.35)),
                    new WaitCommand(600),
                    new InstantCommand(robot.m_dropper::drop),
                    new InstantCommand(() -> robot.m_drive.setMotorPowers(0, 0, 0, 0)),
                    new WaitCommand(500),
                    new InstantCommand(() -> robot.m_lift.setRelativePosition(400)),
                    new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(1450)),
                    new InstantCommand(robot.m_dropper::back),
                    new FollowTrajSequence(robot.m_drive, stackTraj),
                    new InstantCommand(() -> {
                        robot.m_intake.setExtPosition(0.319);
                        robot.m_intake.setSpeed(0.3);
                    }),
                    new InstantCommand(robot.m_intake::suck),
                    new InstantCommand(robot.m_conveyor::up),
                    new WaitCommand(1500),
                    new InstantCommand(robot.m_intake::up),
                    new InstantCommand(() -> robot.m_intake.setSpeed(1)),
                        new InstantCommand(robot.m_intake::suck),
                        new InstantCommand(() -> robot.m_drive.setMotorPowers(0.18, 0.18, 0.18, 0.18)),
                    new WaitCommand(370),
                    new InstantCommand(() -> robot.m_drive.setMotorPowers(0, 0, 0, 0)),
                    new WaitCommand(1500),
                    new FollowTrajSequence(robot.m_drive, asd),
                    new InstantCommand(() -> robot.m_lift.setRelativePosition(1500)),
                    new InstantCommand(robot.m_conveyor::stop),
                    new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(1500)),
                    new InstantCommand(() -> robot.m_drive.setMotorPowers(-0.35, -0.35, -0.35, -0.35)),
                    new WaitCommand(400),
                    new InstantCommand(robot.m_dropper::drop),
                    new InstantCommand(() -> robot.m_drive.setMotorPowers(0, 0, 0, 0)),
                    new WaitCommand(150),
                    new InstantCommand(robot.m_dropper::back),
                        new FollowTrajSequence(robot.m_drive, robot.m_drive.trajectorySequenceBuilder(backboardTraj.end())
                                .addDisplacementMarker(2.3, () -> robot.m_lift.toInitial())
                                .forward(5)
                                .build())
                )
        );
    }
}
