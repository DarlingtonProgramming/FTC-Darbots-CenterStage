package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

public class HookSubsystem extends SubsystemBase {
    private final Servo hook;

    public HookSubsystem(final HardwareMap hwMap, final String name) {
        hook = hwMap.get(Servo.class, name);
        hook.setDirection(DarvinciSettings.HOOK_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public void in() {
        hook.setPosition(DarvinciSettings.HOOK_POS_IN);
    }

    public void out() {
        hook.setPosition(DarvinciSettings.HOOK_POS_OUT);
    }

    public void setPosition(double newPos) {
        hook.setPosition(newPos);
    }

    public void setRelPosition(double newRelPos) {
        hook.setPosition(hook.getPosition() + newRelPos);
    }

    public double getPosition() {
        return hook.getPosition();
    }
}