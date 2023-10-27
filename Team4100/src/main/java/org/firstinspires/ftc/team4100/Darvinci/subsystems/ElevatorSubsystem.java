package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

public class ElevatorSubsystem extends SubsystemBase {
    private final DcMotorEx elevator;
    private final int INITIAL_POS;

    public ElevatorSubsystem(final HardwareMap hwMap, final String name) {
        elevator = hwMap.get(DcMotorEx.class, name);
        INITIAL_POS = elevator.getCurrentPosition();
        elevator.setDirection(DarvinciSettings.ELEVATOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        elevator.setTargetPosition(INITIAL_POS);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elevator.setPower(DarvinciSettings.DEFAULT_ELEVATOR_SPEED);
    }

    public void rise() {
        elevator.setTargetPosition(INITIAL_POS + DarvinciSettings.ELEVATOR_POS_TOP);
    }

    public void lower() {
        elevator.setTargetPosition(INITIAL_POS);
    }

    public void setPosition(int newPos) {
        elevator.setTargetPosition(newPos);
    }

    public void setRelPosition(int newRelPos) {
        elevator.setTargetPosition(elevator.getCurrentPosition() + newRelPos);
    }

    public int getPosition() {
        return elevator.getCurrentPosition();
    }

    public int getTargetPosition() {
        return elevator.getTargetPosition();
    }
}