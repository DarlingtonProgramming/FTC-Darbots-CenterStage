package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Red [L, LP]", group = "Competition", preselectTeleOp = "TeleOp Red")
public class RedLeftLeftParking extends RedLeft {
    public RedLeftLeftParking() {
        super(Parking.LEFT);
    }
}
