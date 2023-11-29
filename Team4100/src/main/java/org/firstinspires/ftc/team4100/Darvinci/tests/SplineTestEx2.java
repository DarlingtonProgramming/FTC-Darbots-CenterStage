package org.firstinspires.ftc.team4100.Darvinci.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team4100.Darvinci.DarvinciAutoBase;
import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;
import org.firstinspires.ftc.team4100.Darvinci.commands.FollowTrajSequence;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyAutoBase;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;

@Autonomous
public class SplineTestEx2 extends DarvinciAutoBase {
        public static final Pose2d m_poseEstimate = new Pose2d(15.29, 63.10, Math.toRadians(90.00));

    public SplineTestEx2() {
        super(DarvinciCore.AllianceType.BLUE, m_poseEstimate);
    }

    @Override
    public void initAuto() {
        schedule(
                new SequentialCommandGroup(
                        new FollowTrajSequence(robot.m_drive,
                                robot.m_drive.trajectorySequenceBuilder(m_poseEstimate)
                                        .lineTo(new Vector2d(12.58, 34.45))
                                        .splineToLinearHeading(new Pose2d(48, 36.05, Math.toRadians(180.00)), Math.toRadians(0.00))
                                        .splineToConstantHeading(new Vector2d(27.45, 10.75), Math.toRadians(180.00))
                                        .splineToConstantHeading(new Vector2d(-58.00, 11.63), Math.toRadians(180.00))
                                        .waitSeconds(0.1)
                                        .lineToSplineHeading(new Pose2d(27.45, 10.75, Math.toRadians(180)))
                                        .splineToLinearHeading(new Pose2d(48, 36.05, Math.toRadians(180.00)), Math.toRadians(0.00))
                                        .build()
                        )
                )
        );
    }
}
