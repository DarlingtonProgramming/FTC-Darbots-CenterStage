package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSystem {
    enum IntakeStatus {
        SPITTING,
        SUCKING,
        IDLE
    }

    private DcMotorEx intake;
    private IntakeStatus status;
    private double speed = 1;

    public IntakeSystem(HardwareMap hwMap) {
        this.intake = hwMap.get(DcMotorEx.class, "Intake");
        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getSpeed() {
        return this.speed;
    }

    public void setSpeed(double newSpeed) {
        this.speed = newSpeed;
    }

    public IntakeStatus getStatus() {
        return status;
    }

    public void setStatus(IntakeStatus newStatus) {
        if (newStatus == IntakeStatus.SPITTING) {
            this.intake.setPower(-this.speed);
        } else if (newStatus == IntakeStatus.SUCKING) {
            this.intake.setPower(this.speed);
        } else {
            this.intake.setPower(0);
        }

        this.status = newStatus;
    }


}
