package org.firstinspires.ftc.team4100.Scrappy.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team4100.Scrappy.commands.FollowTrajSequence;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyAutoBase;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;

@Autonomous
public class SplineTestEx extends ScrappyAutoBase {
    public static final Pose2d m_poseEstimate = new Pose2d(15.29, 61.5, Math.toRadians(270));

    public SplineTestEx() {
        super(ScrappyCore.AllianceType.BLUE, m_poseEstimate);
    }

    @Override
    public void initAuto() {
        schedule(
                new SequentialCommandGroup(
                        new FollowTrajSequence(robot.m_drive,
                                robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                                        .lineTo(new Vector2d(12.58, 34.45))
                                        .waitSeconds(1)
                                        .lineToSplineHeading(new Pose2d(48, 36.05, Math.toRadians(180)))
                                        .waitSeconds(1)
//                                        .lineTo(new Vector2d(27.45, 10.75))
//                                        .lineTo(new Vector2d(-58.00, 11.63))
//                                        .lineTo(new Vector2d(27.45, 10.75))
//                                        .lineTo(new Vector2d(48, 36.05))
//                                        .lineTo(new Vector2d(27.45, 10.75))
//                                        .lineTo(new Vector2d(-58.00, 11.63))
//                                        .lineTo(new Vector2d(27.45, 10.75))
//                                        .lineTo(new Vector2d(48, 36.05))
                                        .build()
                        )
                )
        );
    }
}
