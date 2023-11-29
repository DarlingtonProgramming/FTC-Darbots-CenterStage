package org.firstinspires.ftc.team4100.Scrappy.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {
    public static int LIFT_TOLERANCE = 6;
    public static PIDCoefficients PID_COEFFICIENTS = new PIDCoefficients();
    private final DcMotorEx m_leftMotor, m_rightMotor;

    public LiftSubsystem(final HardwareMap hwMap) {
        m_leftMotor = hwMap.get(DcMotorEx.class, "LSlide");
        m_rightMotor = hwMap.get(DcMotorEx.class, "RSlide");

        m_leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        m_leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_leftMotor.setTargetPosition(0);
        m_rightMotor.setTargetPosition(0);

        m_leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m_leftMotor.setPower(1);
        m_rightMotor.setPower(1);

        m_leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void toInitial() {
        m_leftMotor.setTargetPosition(0);
        m_rightMotor.setTargetPosition(0);
    }

    public void setPosition(int pos) {
        m_leftMotor.setTargetPosition(pos);
        m_rightMotor.setTargetPosition(pos);
    }

    public int getTargetPosition() {
     return m_leftMotor.getTargetPosition();
    }

    public void setRelativePosition(int pos) {
        m_leftMotor.setTargetPosition(m_leftMotor.getTargetPosition() + pos);
        m_rightMotor.setTargetPosition(m_rightMotor.getTargetPosition() + pos);
    }

    public int getPosition() {
        return m_leftMotor.getCurrentPosition();
    }

    public double getVelocity() {
        return m_leftMotor.getVelocity();
    }

    public void setPower(double power) {
        m_leftMotor.setPower(power);
        m_rightMotor.setPower(power);
    }

    public boolean isWithinTolerance(double target) {
        return Math.abs(target - m_leftMotor.getCurrentPosition()) <= LIFT_TOLERANCE;
    }
}