package org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

@Config
public class DriveConstants {

    public static final double TICKS_PER_REV = DarvinciSettings.CHASSIS_MOTOR_TICKS_PER_REV;
    public static final double MAX_RPM = DarvinciSettings.CHASSIS_MOTOR_MAX_RPM;

    public static final boolean RUN_USING_ENCODER = DarvinciSettings.CHASSIS_MOTOR_USE_ENCODERS;
    public static PIDFCoefficients MOTOR_VELO_PID = DarvinciSettings.CHASSIS_MOTOR_PIDF;

    public static double WHEEL_RADIUS = DarvinciSettings.CHASSIS_WHEEL_RADIUS;
    public static double GEAR_RATIO = DarvinciSettings.CHASSIS_GEAR_RATIO;
    public static double TRACK_WIDTH = DarvinciSettings.CHASSIS_TRACK_WIDTH_CALCULATED;

    public static double kV = DarvinciSettings.CHASSIS_kV;
    public static double kA = DarvinciSettings.CHASSIS_kA;
    public static double kStatic = DarvinciSettings.CHASSIS_kStatic;

    public static double MAX_VEL = DarvinciSettings.CHASSIS_MAX_VEL;
    public static double MAX_ACCEL = DarvinciSettings.CHASSIS_MAX_ACCEL;
    public static double MAX_ANG_VEL = DarvinciSettings.CHASSIS_MAX_ANGULAR_VEL_RAD;
    public static double MAX_ANG_ACCEL = DarvinciSettings.CHASSIS_MAX_ANGULAR_ACCEL_RAD;

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = DarvinciSettings.HUB_LOGO_FACING_DIR;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = DarvinciSettings.HUB_USB_FACING_DIR;

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