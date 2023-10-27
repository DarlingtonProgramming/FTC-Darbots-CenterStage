package org.firstinspires.ftc.team4100.Darvinci.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class PushUpLoop extends LinearOpMode {
    private DcMotorEx Elevator;
    private int ELEVATOR_INITIAL;

    @Override
    public void runOpMode() {
        this.Elevator = hardwareMap.get(DcMotorEx.class, "Elevator");
        ELEVATOR_INITIAL = this.Elevator.getCurrentPosition();
        this.Elevator.setTargetPosition(ELEVATOR_INITIAL);
        this.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.Elevator.setPower(1);

        waitForStart();

        while (opModeIsActive()) {
            int newPos = this.Elevator.getTargetPosition() == ELEVATOR_INITIAL ? (ELEVATOR_INITIAL - 3200) : ELEVATOR_INITIAL;

            while (this.Elevator.isBusy()) {
                sleep(10);
            }

            sleep(1000);
            this.Elevator.setTargetPosition(newPos);
        }
    }
}
