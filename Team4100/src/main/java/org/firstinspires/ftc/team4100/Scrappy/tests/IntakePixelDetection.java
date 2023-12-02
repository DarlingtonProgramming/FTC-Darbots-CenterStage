package org.firstinspires.ftc.team4100.Scrappy.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@TeleOp

public class IntakePixelDetection extends LinearOpMode {
    private DcMotorEx intake;
    private boolean enabled = false;
    private long lastDriverRunTime = System.currentTimeMillis();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        Gamepad currentDriverOneGamepad = new Gamepad();
        Gamepad previousDriverOneGamepad = new Gamepad();

        while (opModeIsActive()) {
            try {
                previousDriverOneGamepad.copy(currentDriverOneGamepad);
                currentDriverOneGamepad.copy(gamepad1);
            } catch (Error ignored) {}

            double intakeVoltage = intake.getCurrent(CurrentUnit.AMPS);

            if (currentDriverOneGamepad.b && !previousDriverOneGamepad.b) {
                enabled = !enabled;
                lastDriverRunTime = System.currentTimeMillis();
            }

            if (intakeVoltage > 1.25 && System.currentTimeMillis() - lastDriverRunTime > 400) {
                enabled = false;
            }

            if (enabled) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("voltageConsumed", intakeVoltage);
            telemetry.update();
        }
    }
}
