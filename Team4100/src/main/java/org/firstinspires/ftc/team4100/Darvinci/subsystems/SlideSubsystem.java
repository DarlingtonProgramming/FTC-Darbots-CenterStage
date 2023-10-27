package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

public class SlideSubsystem extends SubsystemBase {
    private final DcMotorEx slide;
    private final int INITIAL_POS;

    public SlideSubsystem(final HardwareMap hwMap, final String name) {
        slide = hwMap.get(DcMotorEx.class, name);
        slide.setDirection(DarvinciSettings.SLIDE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        INITIAL_POS = slide.getCurrentPosition() + DarvinciSettings.SLIDE_OFFSET;
        slide.setTargetPosition(INITIAL_POS);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(DarvinciSettings.DEFAULT_SLIDE_SPEED);
    }

    public void setPosition(int newPos) {
        slide.setTargetPosition(newPos);
    }

    public void setRelPosition(int newRelPos) {
        slide.setTargetPosition(slide.getCurrentPosition() + newRelPos);
    }

    public void setInitialRel(int newRelPos) {
        slide.setTargetPosition(INITIAL_POS + newRelPos);
    }

    public int getPosition() {
        return slide.getCurrentPosition();
    }

    public int getRelPosition() {
        return INITIAL_POS - slide.getCurrentPosition();
    }

    public int getTargetPosition() {
        return slide.getTargetPosition();
    }

}