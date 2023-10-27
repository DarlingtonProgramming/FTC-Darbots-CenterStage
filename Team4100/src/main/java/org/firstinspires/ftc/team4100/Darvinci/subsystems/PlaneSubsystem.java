package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

public class PlaneSubsystem extends SubsystemBase {
    private final Servo plane;

    public PlaneSubsystem(final HardwareMap hwMap, final String name) {
        plane = hwMap.get(Servo.class, name);
        plane.setDirection(DarvinciSettings.PLANE_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        this.hold();
    }

    public void shoot() {
        plane.setPosition(DarvinciSettings.PLANE_POS_OUT);
    }

    public void hold() {
        plane.setPosition(DarvinciSettings.PLANE_POS_IN);
    }

    public void setPosition(double newPos) {
        plane.setPosition(newPos);
    }

    public void setRelPosition(double newRelPos) {
        plane.setPosition(plane.getPosition() + newRelPos);
    }

    public double getPosition() {
        return plane.getPosition();
    }
}