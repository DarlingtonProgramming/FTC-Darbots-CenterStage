package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red [R, RP]", group = "Competition", preselectTeleOp = "TeleOp Red")
public class RedRightRightParking extends RedRight {
    public RedRightRightParking() {
        super(Parking.RIGHT);
    }
}
