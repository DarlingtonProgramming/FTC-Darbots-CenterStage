package org.firstinspires.ftc.team4100.Darvinci.teleop.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

import java.util.List;

@TeleOp(name="Old TeleOp - Regular", group="old")
@Disabled
public class DarvinciTeleOpOld extends LinearOpMode {
    private FtcDashboard Dashboard;
    private List<LynxModule> Hubs;
    private DcMotorEx LF, LB, RF, RB;
    private DcMotorEx Slide, Intake, Elevator;
    private Servo Outtake, Hook, Plane;
    private CRServo Push;
    private double speed = 0.9;

    private int SLIDE_INITIAL;
    private int slideTarget;
    private int ELEVATOR_INITIAL;

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
        SLIDE_INITIAL = this.Slide.getCurrentPosition() + 25;
        slideTarget = SLIDE_INITIAL + 700;
        this.Slide.setTargetPosition(SLIDE_INITIAL);
        this.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Slide.setPower(1);

        this.Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        this.Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.Outtake = hardwareMap.get(Servo.class, "Outtake");
        this.Outtake.setPosition(DarvinciSettings.BUCKET_POS_IN);
        this.Push = hardwareMap.get(CRServo.class, "Push");
        this.Push.setDirection(DcMotorSimple.Direction.REVERSE);

        this.Elevator = hardwareMap.get(DcMotorEx.class, "Elevator");
        ELEVATOR_INITIAL = this.Elevator.getCurrentPosition();
        this.Elevator.setTargetPosition(ELEVATOR_INITIAL);
        this.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.Elevator.setPower(1);
        this.Hook = hardwareMap.get(Servo.class, "Hook");
        this.Plane = hardwareMap.get(Servo.class, "Plane");

        Gamepad currentDriverOneGamepad = new Gamepad();
        Gamepad currentDriverTwoGamepad = new Gamepad();
        Gamepad previousDriverOneGamepad = new Gamepad();
        Gamepad previousDriverTwoGamepad = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {
            try {
                previousDriverOneGamepad.copy(currentDriverOneGamepad);
                currentDriverOneGamepad.copy(gamepad1);

                previousDriverTwoGamepad.copy(currentDriverTwoGamepad);
                currentDriverTwoGamepad.copy(gamepad2);
            } catch (Error ignored) {}

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (Elevator.getPower() > 0.8 && Elevator.getTargetPosition() < (ELEVATOR_INITIAL + 3000) && (Elevator.getCurrentPosition() > (ELEVATOR_INITIAL + 50) && Elevator.getCurrentPosition() < (ELEVATOR_INITIAL + 150))) {
                Elevator.setPower(0.05);
            }

            // Driver Two
            if (currentDriverTwoGamepad.a && !previousDriverTwoGamepad.a) {
                this.Elevator.setTargetPosition(ELEVATOR_INITIAL);
            }

            if (currentDriverTwoGamepad.y && !previousDriverTwoGamepad.y) {
                this.Elevator.setPower(1);
                this.Elevator.setTargetPosition(ELEVATOR_INITIAL + 3200);
            }

            if (currentDriverTwoGamepad.x && !previousDriverTwoGamepad.x) {
                this.Hook.setPosition(0);
            }

            if (currentDriverTwoGamepad.b && !previousDriverTwoGamepad.b) {
                this.Hook.setPosition(1);
            }

            if (currentDriverTwoGamepad.left_bumper && !previousDriverTwoGamepad.left_bumper) {
                this.Plane.setPosition(0);
            }

            if (currentDriverTwoGamepad.right_bumper && !previousDriverTwoGamepad.right_bumper) {
                this.Plane.setPosition(1);
            }

            if (currentDriverTwoGamepad.dpad_left && !currentDriverTwoGamepad.dpad_left) {
                this.Hook.setPosition(this.Hook.getPosition() - 0.1);
            }

            if (currentDriverTwoGamepad.dpad_right && !currentDriverTwoGamepad.dpad_right) {
                this.Hook.setPosition(this.Hook.getPosition() + 0.1);
            }

            if (currentDriverTwoGamepad.dpad_up && !currentDriverTwoGamepad.dpad_up) {
                this.Plane.setPosition(this.Plane.getPosition() + 0.1);
            }

            if (currentDriverTwoGamepad.dpad_down && !currentDriverTwoGamepad.dpad_down) {
                this.Plane.setPosition(this.Plane.getPosition() - 0.1);
            }

            // Driver One
            if (currentDriverOneGamepad.dpad_left && !previousDriverOneGamepad.dpad_left) {
                this.Outtake.setPosition(this.Outtake.getPosition() - 0.05);
            }

            if (currentDriverOneGamepad.dpad_right && !previousDriverOneGamepad.dpad_right) {
                this.Outtake.setPosition(this.Outtake.getPosition() + 0.05);
            }

            if (currentDriverOneGamepad.y && !previousDriverOneGamepad.y) {
                if (Outtake.getPosition() > DarvinciSettings.BUCKET_POS_OUT) {
                    new Thread(() -> {
                        sleep(600);
                        this.Outtake.setPosition(DarvinciSettings.BUCKET_POS_OUT);
                    }).start();
                }
                this.Slide.setTargetPosition(slideTarget);
            } else if (currentDriverOneGamepad.x && !previousDriverOneGamepad.x) {
                this.Outtake.setPosition(DarvinciSettings.BUCKET_POS_IN - 0.13);
                sleep(200);
                this.Slide.setTargetPosition(SLIDE_INITIAL + 75);
                new Thread(() -> {
                    while (Slide.getCurrentPosition() > (SLIDE_INITIAL + 95)) {
                        sleep(10);
                    }
                    sleep(100);
                    this.Outtake.setPosition(DarvinciSettings.BUCKET_POS_IN);
                    sleep(500);
                    this.Slide.setTargetPosition(SLIDE_INITIAL + 25);
                }).start();
            } else if (currentDriverOneGamepad.b && !previousDriverOneGamepad.b && !currentDriverOneGamepad.start) {
                this.speed = 0.9;
            } else if (currentDriverOneGamepad.a && !previousDriverOneGamepad.a && !currentDriverOneGamepad.start) {
                this.speed = 0.3;
            } else if (currentDriverOneGamepad.dpad_up && !previousDriverOneGamepad.dpad_up) {
                this.Slide.setTargetPosition(Slide.getCurrentPosition() + 150);
                if (Slide.getTargetPosition() > slideTarget) {
                    slideTarget = Slide.getTargetPosition();
                }
            } else if (currentDriverOneGamepad.dpad_down && !previousDriverOneGamepad.dpad_down) {
                this.Slide.setTargetPosition(Slide.getCurrentPosition() - 150);
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
            this.telemetry.addData("Slide Rel", (this.Slide.getCurrentPosition() - SLIDE_INITIAL));
            this.telemetry.addData("Outtake", this.Outtake.getPosition());
            this.telemetry.addData("Elevator", this.Elevator.getCurrentPosition());
            this.telemetry.addData("Hook", this.Hook.getPosition());
            this.telemetry.update();
        }
    }
}

