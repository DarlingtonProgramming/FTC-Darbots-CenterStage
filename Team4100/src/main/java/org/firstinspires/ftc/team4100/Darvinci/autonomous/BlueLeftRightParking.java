package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Blue [L, RP]", group = "Competition", preselectTeleOp = "TeleOp Blue")
public class BlueLeftRightParking extends BlueLeft {
    public BlueLeftRightParking() {
        super(Parking.RIGHT);
    }
}
