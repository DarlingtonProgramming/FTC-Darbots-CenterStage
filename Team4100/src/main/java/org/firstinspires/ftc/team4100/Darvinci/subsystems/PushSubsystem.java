package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

public class PushSubsystem extends SubsystemBase {
    private final CRServo push;
    private double speed = DarvinciSettings.DEFAULT_PUSH_SPEED;

    public PushSubsystem(final HardwareMap hwMap, final String name) {
        push = hwMap.get(CRServo.class, name);
        push.setDirection(DarvinciSettings.PUSH_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        this.stop();
    }

    public void setPower(double power) {
        push.setPower(power);
    }

    public double getPower() {
        return push.getPower();
    }

    public void setSpeed(double newSpeed) {
        speed = newSpeed;
    }

    public double getSpeed() {
        return speed;
    }

    public void spit() {
        push.setPower(speed);
    }

    public void suck() {
        push.setPower(-speed);
    }

    public void stop() {
        push.setPower(0);
    }
}