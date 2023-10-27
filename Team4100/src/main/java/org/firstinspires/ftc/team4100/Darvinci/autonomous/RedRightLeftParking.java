package org.firstinspires.ftc.team4100.Darvinci.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red [R, LP]", group="Competition")
public class RedRightLeftParking extends RedRight {
    public RedRightLeftParking() {
        super(Parking.LEFT);
    }
}
