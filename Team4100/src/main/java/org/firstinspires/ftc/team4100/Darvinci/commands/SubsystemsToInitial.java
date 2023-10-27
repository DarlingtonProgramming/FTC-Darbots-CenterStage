package org.firstinspires.ftc.team4100.Darvinci.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.team4100.Darvinci.subsystems.BucketSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.SlideSubsystem;

public class SubsystemsToInitial extends SequentialCommandGroup {
    public SubsystemsToInitial(BucketSubsystem bucket, SlideSubsystem slide) {
        addCommands(
                new InstantCommand(() -> bucket.inRel(-0.13)),
                new WaitCommand(250),
                new InstantCommand(() -> slide.setInitialRel(75)),
                new WaitUntilCommand(() -> slide.getRelPosition() < 95),
                new InstantCommand(bucket::in),
                new WaitCommand(500),
                new InstantCommand(() -> slide.setInitialRel(-25))
        );
        addRequirements(bucket, slide);
    }
}
