package org.firstinspires.ftc.team4100.Scrappy.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.team4100.FieldConstant;
import org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive.ScrappyChassis;
import org.firstinspires.ftc.team4100.Scrappy.subsystems.IntakeSubsystem;

public class DriveAndIntakeStack extends SequentialCommandGroup {
    public DriveAndIntakeStack(ScrappyChassis drive, IntakeSubsystem intake) {
        addCommands(
                new ParallelCommandGroup(
                        new DriveToAprilTag(drive, 9, 8),
                        new SequentialCommandGroup(
                                new WaitCommand(350),
                                new InstantCommand(intake::down)
                        )
                ),
                new InstantCommand(intake::suck),
                new WaitCommand(2000),
                new InstantCommand(intake::down),
                new WaitCommand(2000),
                new InstantCommand(intake::up),
                new FollowTrajSequence(drive,
                        drive.trajectorySequenceBuilder(new Pose2d(FieldConstant.getAprilTagPose(9), Math.toRadians(180)))
                                .forward(3)
                                .build()
                ),
                new WaitCommand(2000),
                new InstantCommand(intake::stop)
        );
    }
}
