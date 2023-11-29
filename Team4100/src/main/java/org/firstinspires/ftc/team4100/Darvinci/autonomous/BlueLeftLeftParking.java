package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Blue [L, LP]", group = "Competition", preselectTeleOp = "TeleOp Blue")
public class BlueLeftLeftParking extends BlueLeft {
    public BlueLeftLeftParking() {
        super(Parking.LEFT);
    }
}
