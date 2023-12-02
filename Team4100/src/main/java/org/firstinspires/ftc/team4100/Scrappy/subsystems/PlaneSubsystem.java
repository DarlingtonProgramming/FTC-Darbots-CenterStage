package org.firstinspires.ftc.team4100.Scrappy.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PlaneSubsystem extends SubsystemBase {
    public final static double LAUNCH_POS = 0.85;
    public final static double BACK_POS = 0.75;
    private final Servo m_plane;

    public PlaneSubsystem(final HardwareMap hwMap) {
        m_plane = hwMap.get(Servo.class, "Plane");
    }

    public void launch() {
        m_plane.setPosition(LAUNCH_POS);
    }

    public void launch(double pos) {
        m_plane.setPosition(LAUNCH_POS + pos);
    }

    public void back() {
        m_plane.setPosition(BACK_POS);
    }

    public void back(double pos) {
        m_plane.setPosition(BACK_POS + pos);
    }

    public void setPosition(double pos) {
        m_plane.setPosition(pos);
    }

    public void setRelativePosition(double pos) {
        m_plane.setPosition(m_plane.getPosition() + pos);
    }

    public double getPosition() {
        return m_plane.getPosition();
    }
}
