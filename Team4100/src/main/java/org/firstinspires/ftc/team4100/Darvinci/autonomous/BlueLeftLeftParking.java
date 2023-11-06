package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue [L, LP]", group = "Competition", preselectTeleOp = "TeleOp Blue")
public class BlueLeftLeftParking extends BlueLeft {
    public BlueLeftLeftParking() {
        super(Parking.LEFT);
    }
}
