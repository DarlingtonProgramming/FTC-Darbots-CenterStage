package org.firstinspires.ftc.team4100.Darvinci.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp

public class bro2 extends LinearOpMode {
    private DcMotorEx a, b;

    @Override
    public void runOpMode() throws InterruptedException {
        a = hardwareMap.get(DcMotorEx.class, "left");
        b = hardwareMap.get(DcMotorEx.class, "right");

        a.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // left is reversed

        waitForStart();

        while (opModeIsActive()) {
            a.setPower(0.9);
            b.setPower(-0.9);
        }
    }
}
