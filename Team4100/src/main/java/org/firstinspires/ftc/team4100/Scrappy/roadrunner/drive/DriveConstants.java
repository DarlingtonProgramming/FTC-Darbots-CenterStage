package org.firstinspires.ftc.team4100.Scrappy.roadrunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    public static double WHEEL_RADIUS = 1.8898;
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 13.1;
    public static double kV = 0.01269;
    public static double kA = 0.0044;
    public static double kStatic = 0.0022;
    public static double MAX_VEL = 65;
    public static double MAX_ACCEL = 65;
    public static double MAX_ANG_VEL = Math.toRadians(270);
    public static double MAX_ANG_ACCEL = Math.toRadians(270);

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