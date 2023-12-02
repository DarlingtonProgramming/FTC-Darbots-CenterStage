package org.firstinspires.ftc.team5100.MolotovCock.roadrunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {

    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.8898;
    public static double GEAR_RATIO = 1 * (37.8 / 37.34644554178704);
    public static double TRACK_WIDTH = 15.25;

    public static double kV = 0.0161;
    public static double kA = 0.0029;
    public static double kStatic = 0;

    public static double MAX_VEL = 45.48291908330528;
    public static double MAX_ACCEL = 45.48291908330528;
    public static double MAX_ANG_VEL = Math.toRadians(252.92903568306565);
    public static double MAX_ANG_ACCEL = Math.toRadians(252.92903568306565);

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}