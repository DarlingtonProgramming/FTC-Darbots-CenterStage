package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

public class BucketSubsystem extends SubsystemBase {
    private final Servo bucket;

    public BucketSubsystem(final HardwareMap hwMap, final String name) {
        bucket = hwMap.get(Servo.class, name);
        bucket.setDirection(DarvinciSettings.BUCKET_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        this.in();
    }

    public void out() {
        bucket.setPosition(DarvinciSettings.BUCKET_POS_OUT);
    }

    public void outRel(double pos) {
        bucket.setPosition(DarvinciSettings.BUCKET_POS_OUT + pos);
    }

    public void in() {
        bucket.setPosition(DarvinciSettings.BUCKET_POS_IN);
    }

    public void inRel(double pos) {
        bucket.setPosition(DarvinciSettings.BUCKET_POS_IN + pos);
    }

    public void setPosition(double newPos) {
        bucket.setPosition(newPos);
    }

    public void setRelPosition(double newRelPos) {
        bucket.setPosition(bucket.getPosition() + newRelPos);
    }

    public double getPosition() {
        return bucket.getPosition();
    }
}