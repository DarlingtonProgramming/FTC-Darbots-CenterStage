package org.firstinspires.ftc.team4100.Darvinci.teleop.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

import java.util.List;

@Disabled
@TeleOp(name="Old TeleOp - Field Centric gsay", group="old")
public class DarvinciTeleOpFieldCentric extends LinearOpMode {
    private FtcDashboard Dashboard;
    private List<LynxModule> Hubs;
    private IMU imu;
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

        this.imu = hardwareMap.get(IMU.class, "imu");
        this.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(DarvinciSettings.HUB_LOGO_FACING_DIR, DarvinciSettings.HUB_USB_FACING_DIR)));

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
        this.Plane.setPosition(DarvinciSettings.PLANE_POS_IN);

        Gamepad currentDriverOneGamepad = new Gamepad();
        Gamepad currentDriverTwoGamepad = new Gamepad();
        Gamepad previousDriverOneGamepad = new Gamepad();
        Gamepad previousDriverTwoGamepad = new Gamepad();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            try {
                previousDriverOneGamepad.copy(currentDriverOneGamepad);
                currentDriverOneGamepad.copy(gamepad1);

                previousDriverTwoGamepad.copy(currentDriverTwoGamepad);
                currentDriverTwoGamepad.copy(gamepad2);
            } catch (Error ignored) {}

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
                this.Plane.setPosition(DarvinciSettings.PLANE_POS_IN);
            }

            if (currentDriverTwoGamepad.right_bumper && !previousDriverTwoGamepad.right_bumper) {
                this.Plane.setPosition(DarvinciSettings.PLANE_POS_OUT);
            }

            if (currentDriverTwoGamepad.dpad_left && !previousDriverTwoGamepad.dpad_left) {
                this.Hook.setPosition(this.Hook.getPosition() - 0.1);
            }

            if (currentDriverTwoGamepad.dpad_right && !previousDriverTwoGamepad.dpad_right) {
                this.Hook.setPosition(this.Hook.getPosition() + 0.1);
            }

            if (currentDriverTwoGamepad.dpad_up && !previousDriverTwoGamepad.dpad_up) {
                this.Plane.setPosition(this.Plane.getPosition() + 0.05);
            }

            if (currentDriverTwoGamepad.dpad_down && !previousDriverTwoGamepad.dpad_down) {
                this.Plane.setPosition(this.Plane.getPosition() - 0.05);
            }

            // Driver One
            if (currentDriverOneGamepad.back && !previousDriverOneGamepad.back) {
                this.imu.resetYaw();
            }

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
                    this.Slide.setTargetPosition(SLIDE_INITIAL);
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

            // Drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double LFPower = (rotY + rotX + rx) / denominator;
            double LBPower = (rotY - rotX + rx) / denominator;
            double RFPower = (rotY - rotX - rx) / denominator;
            double RBPower = (rotY + rotX - rx) / denominator;

            this.LF.setPower(this.speed * LFPower);
            this.LB.setPower(this.speed * LBPower);
            this.RF.setPower(this.speed * RFPower);
            this.RB.setPower(this.speed * RBPower);

            this.telemetry.addData("Slide", this.Slide.getCurrentPosition());
            this.telemetry.addData("Slide Rel", (this.Slide.getCurrentPosition() - SLIDE_INITIAL));
            this.telemetry.addData("Outtake", this.Outtake.getPosition());
            this.telemetry.addData("Elevator", this.Elevator.getCurrentPosition());
            this.telemetry.addData("Hook", this.Hook.getPosition());
            this.telemetry.addData("Plane", this.Plane.getPosition());
            this.telemetry.update();
        }
    }
}

