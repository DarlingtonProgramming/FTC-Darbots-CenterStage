package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue [L, RP]", group="Competition")
public class BlueLeftRightParking extends BlueLeft {
    public BlueLeftRightParking() {
        super(Parking.RIGHT);
    }
}
