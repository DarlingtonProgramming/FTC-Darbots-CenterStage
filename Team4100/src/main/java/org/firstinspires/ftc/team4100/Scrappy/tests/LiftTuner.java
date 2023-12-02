package org.firstinspires.ftc.team4100.Scrappy.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Config
@Autonomous
public class LiftTuner extends LinearOpMode {
    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private DcMotorEx leftMotor, rightMotor;
    public static double MAX_RPM = 435;
    public static double TICKS_PER_REV = 384.5;
    public static double MAX_VEL = 35;
    public static double MAX_ACCEL = 35;

    public static double kV = 1 / MAX_VEL;
    // f 11.17
    // p 15
    public static PIDFCoefficients PIDF = new PIDFCoefficients(0, 0, 0, 32767 / (MAX_RPM / 60 * TICKS_PER_REV));
    public static double target = 48;

    private static MotionProfile generateProfile(boolean movingUp) {
        MotionState start = new MotionState(movingUp ? 0 : target, 0, 0, 0);
        MotionState goal = new MotionState(movingUp ? target : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL, 0);
    }

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotorEx.class, "LSlide");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotorEx.class, "RSlide");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Mode mode = Mode.TUNING_MODE;

        double lastKp = PIDF.p;
        double lastKi = PIDF.i;
        double lastKd = PIDF.d;
        double lastKf = PIDF.f;

        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();

        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
                    double targetPower = kV * motionState.getV();
                    leftMotor.setPower(targetPower);
                    rightMotor.setPower(targetPower);

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());

                    double leftMotorVel = 2 * Math.PI * leftMotor.getVelocity() / TICKS_PER_REV;
                    double rightMotorVel = 2 * Math.PI * rightMotor.getVelocity() / TICKS_PER_REV;

                    telemetry.addData("measuredVelocity0", leftMotorVel);
                    telemetry.addData("error0", motionState.getV() - leftMotorVel);

                    telemetry.addData("measuredVelocity1", rightMotorVel);
                    telemetry.addData("error1", motionState.getV() - rightMotorVel);
                    break;
                case DRIVER_MODE:
                    if (gamepad1.b) {
                        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
            }

            if (lastKp != PIDF.p || lastKd != PIDF.d
                    || lastKi != PIDF.i || lastKf != PIDF.f) {
                leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
                rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

                lastKp = PIDF.p;
                lastKi = PIDF.i;
                lastKd = PIDF.d;
                lastKf = PIDF.f;
            }

            telemetry.update();
        }
    }
}
