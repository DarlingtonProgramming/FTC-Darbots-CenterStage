package org.firstinspires.ftc.team4100.Darvinci.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;

//@Config
@Disabled
@TeleOp(name="TeleOp Red", group="Competition")
public class DarvinciTeleOpRed extends DarvinciTeleOp {
    public static double offset = Math.toRadians(180);
    public DarvinciTeleOpRed() {
        super(DarvinciCore.AllianceType.RED, offset);
    }
}
