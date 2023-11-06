package org.firstinspires.ftc.team4100.Darvinci.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciAutonomousSettings;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.BucketSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.PushSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.SlideSubsystem;

public class DumpOnBackboard extends SequentialCommandGroup {
    public DumpOnBackboard(BucketSubsystem bucket, PushSubsystem push, SlideSubsystem slide) {
        addCommands(
                new InstantCommand(() -> bucket.outRel(-0.01)),
                new WaitCommand(1050),
                new InstantCommand(push::spit),
                new WaitCommand(2900),
                new InstantCommand(() -> slide.setRelPosition(DarvinciAutonomousSettings.SLIDE_ENSURE_PIXEL_FACTOR)),
                new WaitCommand(1000),
                new InstantCommand(() -> bucket.setRelPosition(-0.01)),
                new WaitCommand(250),
                new InstantCommand(() -> slide.setRelPosition(DarvinciAutonomousSettings.SLIDE_ENSURE_PIXEL_FACTOR)),
                new WaitCommand(1000),
                new InstantCommand(push::stop)
        );
        addRequirements(bucket, push, slide);
    }
}
