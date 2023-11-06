package org.firstinspires.ftc.team4100.Darvinci.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;

@Config
@TeleOp(name="TeleOp Blue", group="Competition")
public class DarvinciTeleOpBlue extends DarvinciTeleOp {
    public static double offset = Math.toRadians(0);
    public DarvinciTeleOpBlue() {
        super(DarvinciCore.AllianceType.BLUE, offset);
    }
}
