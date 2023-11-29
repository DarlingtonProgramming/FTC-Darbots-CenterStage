package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Red [L, RP]", group = "Competition", preselectTeleOp = "TeleOp Red")
public class RedLeftRightParking extends RedLeft {
    public RedLeftRightParking() {
        super(Parking.RIGHT);
    }
}
