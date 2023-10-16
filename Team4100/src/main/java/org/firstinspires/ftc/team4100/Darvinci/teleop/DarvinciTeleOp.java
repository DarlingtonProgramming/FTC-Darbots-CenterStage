package org.firstinspires.ftc.team4100.Darvinci.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name="Darvinci - TeleOp")
public class DarvinciTeleOp extends LinearOpMode {
    private FtcDashboard Dashboard;
    private List<LynxModule> Hubs;
    private DcMotorEx LF, LB, RF, RB;
    private DcMotorEx Slide, Intake;
    private Servo Outtake;
    private CRServo Push;
    private double speed = 1;

    private int SLIDE_INITIAL;

    @Override
    public void runOpMode() {
        this.Dashboard = FtcDashboard.getInstance();
        this.Hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : this.Hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.LF = hardwareMap.get(DcMotorEx.class, "LF");
        this.LB = hardwareMap.get(DcMotorEx.class, "LB");
        this.RF = hardwareMap.get(DcMotorEx.class, "RF");
        this.RB = hardwareMap.get(DcMotorEx.class, "RB");

        this.RB.setDirection(DcMotorSimple.Direction.REVERSE);
        this.RF.setDirection(DcMotorSimple.Direction.REVERSE);

        this.LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.Slide = hardwareMap.get(DcMotorEx.class, "Slide");
        this.Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        SLIDE_INITIAL = this.Slide.getCurrentPosition() + 75;
        this.Slide.setTargetPosition(SLIDE_INITIAL);
        this.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Slide.setPower(1);

        this.Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        this.Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.Outtake = hardwareMap.get(Servo.class, "Outtake");
        this.Push = hardwareMap.get(CRServo.class, "Push");

        Gamepad currentDriverOneGamepad = new Gamepad();
        Gamepad previousDriverOneGamepad = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {
            try {
                previousDriverOneGamepad.copy(currentDriverOneGamepad);
                currentDriverOneGamepad.copy(gamepad1);
            } catch (Error ignored) {}

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (currentDriverOneGamepad.dpad_left && !previousDriverOneGamepad.dpad_left) {
                this.Outtake.setPosition(this.Outtake.getPosition() - 0.05);
            }

            if (currentDriverOneGamepad.dpad_right && !previousDriverOneGamepad.dpad_right) {
                this.Outtake.setPosition(this.Outtake.getPosition() + 0.05);
            }

            if (currentDriverOneGamepad.y && !previousDriverOneGamepad.y) {
                this.Slide.setTargetPosition(this.Slide.getCurrentPosition() + 100);
            }

            if (currentDriverOneGamepad.b && !previousDriverOneGamepad.b) {
                this.Slide.setTargetPosition(SLIDE_INITIAL);
            }

            if (currentDriverOneGamepad.a && !previousDriverOneGamepad.a) {
                this.Slide.setTargetPosition(this.Slide.getCurrentPosition() - 100);
            }

            if (currentDriverOneGamepad.right_bumper) {
                this.Push.setPower(1);
            } else if (currentDriverOneGamepad.left_bumper) {
                this.Push.setPower(-1);
            } else {
                this.Push.setPower(0);
            }

            if (currentDriverOneGamepad.left_trigger > 0.8) {
                this.Intake.setPower(-1);
            } else if (currentDriverOneGamepad.right_trigger > 0.8) {
                this.Intake.setPower(1);
            } else {
                this.Intake.setPower(0);
            }

            double LFPower = Range.clip(this.speed * (drive + rotate - strafe), -1.0, 1.0);
            double LBPower = Range.clip(this.speed * (drive + rotate + strafe), -1.0, 1.0);
            double RFPower = Range.clip(this.speed * (drive - rotate + strafe), -1.0, 1.0);
            double RBPower = Range.clip(this.speed * (drive - rotate - strafe), -1.0, 1.0);

            this.LF.setPower(LFPower);
            this.LB.setPower(LBPower);
            this.RF.setPower(RFPower);
            this.RB.setPower(RBPower);

            this.telemetry.addData("Slide", this.Slide.getCurrentPosition());
            this.telemetry.addData("Outtake", this.Outtake.getPosition());
            this.telemetry.update();
        }
    }
}

