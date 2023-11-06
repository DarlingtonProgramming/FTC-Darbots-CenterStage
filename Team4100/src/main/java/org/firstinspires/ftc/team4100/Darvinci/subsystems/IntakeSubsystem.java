package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intake;
    private double speed = DarvinciSettings.DEFAULT_INTAKE_SPEED;

    public IntakeSubsystem(final HardwareMap hwMap, final String name) {
        intake = hwMap.get(DcMotorEx.class, name);
        intake.setDirection(DarvinciSettings.INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.stop();
    }

    public void setSpeed(double newSpeed) {
        speed = newSpeed;
    }

    public double getSpeed() {
        return speed;
    }

    public void spit() {
        intake.setPower(-speed);
    }

    public void suck() {
        intake.setPower(speed);
    }

    public void stop() {
        intake.setPower(0);
    }

    public boolean isSucking() {
        return intake.getPower() == speed;
    }

    public boolean isSpitting() {
        return intake.getPower() == -speed;
    }

}