package org.firstinspires.ftc.team4100.Scrappy.commands;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.team4100.Scrappy.subsystems.LiftSubsystem;

public class LiftGotoPosition extends CommandBase {
    private final LiftSubsystem m_lift;
    private final double m_targetPosition;
    private final PIDFController m_controller;
    public LiftGotoPosition(final LiftSubsystem lift, double targetPosition) {
        m_lift = lift;
        m_targetPosition = targetPosition;
        m_controller = new PIDFController(LiftSubsystem.PID_COEFFICIENTS);
    }

    @Override
    public void initialize() {
        m_controller.reset();
        m_controller.setTargetPosition(m_targetPosition);
    }

    @Override
    public void execute() {
        m_lift.setPower(m_controller.update(m_lift.getPosition(), m_lift.getVelocity()));
    }

    @Override
    public boolean isFinished() {
        return m_lift.isWithinTolerance(m_targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        m_lift.setPower(0.3);
    }
}
