package org.firstinspires.ftc.team4100.Darvinci.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.trajectorysequence.TrajectorySequence;

public class FollowTrajSequence extends CommandBase {
    private final DarvinciChassis m_rr;
    private final TrajectorySequence m_trajSequence;

    public FollowTrajSequence(DarvinciChassis rr, TrajectorySequence trajSequence) {
        m_rr = rr;
        m_trajSequence = trajSequence;
    }

    @Override
    public void initialize() {
        // async following for other tasks to run
        m_rr.followTrajectorySequenceAsync(m_trajSequence);
    }

    @Override
    public void execute() {
        m_rr.update();
    }

    @Override
    public boolean isFinished() {
        return !m_rr.isBusy();
    }
}
