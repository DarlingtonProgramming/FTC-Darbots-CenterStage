package org.firstinspires.ftc.team4100.Scrappy.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {
    public static double EXT_UP_POS = 0.65;
    public static double EXT_DOWN_POS = 0.12;

    private final DcMotorEx m_intake;
    private final Servo m_extension;
    private double m_intakeSpeed = 1;

    public IntakeSubsystem(final HardwareMap hwMap) {
        m_intake = hwMap.get(DcMotorEx.class, "Intake");
        m_extension = hwMap.get(Servo.class, "IntakeEx");

        m_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.stop();
    }

    public void up() {
        m_extension.setPosition(EXT_UP_POS);
    }

    public void up(double pos) {
        m_extension.setPosition(EXT_UP_POS + pos);
    }

    public void down() {
        m_extension.setPosition(EXT_DOWN_POS);
    }

    public void down(double pos) {
        m_extension.setPosition(EXT_DOWN_POS + pos);
    }

    public void setExtPosition(double pos) {
        m_extension.setPosition(pos);
    }

    public void setExtRelativePosition(double pos) {
        m_extension.setPosition(m_extension.getPosition() + pos);
    }

    public void suck() {
        m_intake.setPower(m_intakeSpeed);
    }

    public void spit() {
        m_intake.setPower(-m_intakeSpeed);
    }

    public void stop() {
        m_intake.setPower(0);
    }

    public void setSpeed(double speed) {
        m_intakeSpeed = Range.clip(speed, 0, 1);
    }

    public double getSpeed() {
        return m_intakeSpeed;
    }

    public double getPower() {
        return m_intake.getPower();
    }

    public double getExtPosition() {
        return m_extension.getPosition();
    }

    public double getCurrent() {
        return m_intake.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isSucking() {
        return m_intake.getPower() == m_intakeSpeed;
    }

    public boolean isSpitting() {
        return m_intake.getPower() == -m_intakeSpeed;
    }
}
