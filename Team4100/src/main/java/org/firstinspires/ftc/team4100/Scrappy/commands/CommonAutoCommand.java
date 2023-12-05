package org.firstinspires.ftc.team4100.Scrappy.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.trajectorysequence.TrajectorySequence;

public class CommonAutoCommand extends SequentialCommandGroup {
    public CommonAutoCommand(ScrappyCore robot, TrajectorySequence detectionTraj, TrajectorySequence backboardTraj) {
        addCommands(
                new SequentialCommandGroup(
                        new FollowTrajSequence(robot.m_drive, detectionTraj),
                        new InstantCommand(() -> robot.m_lift.setRelativePosition(400)),
                        new WaitCommand(1000),
                        new InstantCommand(robot.m_dropper::drop),
                        new WaitCommand(1000),
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
                        new InstantCommand(() -> robot.m_drive.setMotorPowers(-0.2, -0.2, -0.2, -0.2)),
                        new WaitCommand(1350),
                        new InstantCommand(robot.m_dropper::drop),
                        new InstantCommand(() -> robot.m_drive.setMotorPowers(0, 0, 0, 0)),
                        new WaitCommand(1200),
                        new InstantCommand(() -> robot.m_lift.setRelativePosition(550)),
                        new WaitUntilCommand(() -> robot.m_lift.isWithinTolerance(1600)),
                        new InstantCommand(robot.m_dropper::back),
                        new FollowTrajSequence(robot.m_drive, robot.m_drive.trajectorySequenceBuilder(backboardTraj.end())
                                .addDisplacementMarker(2.3, () -> robot.m_lift.toInitial())
                                .forward(5)
                                .build())
                )
        );
    }
}
