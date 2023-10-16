package org.firstinspires.ftc.team4100.Darvinci.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DarvinciSettings {
    //========== Start of Chassis Settings ==========
    public static double CHASSIS_LENGTH = 17.5; // in
    public static double CHASSIS_WIDTH = 16.3; // in

    public static double CHASSIS_GEAR_RATIO = 1;
    public static boolean CHASSIS_MOTOR_USE_ENCODERS = false;

    public static double CHASSIS_MOTOR_TICKS_PER_REV = 537.7;
    public static double CHASSIS_MOTOR_MAX_RPM = 312;
    public static PIDFCoefficients CHASSIS_MOTOR_PIDF = new PIDFCoefficients(0, 0, 0, (32767 / (CHASSIS_MOTOR_MAX_RPM / 60 * CHASSIS_MOTOR_TICKS_PER_REV)));

    public static double CHASSIS_WHEEL_RADIUS = 1.8898; // in
    public static double CHASSIS_TRACK_WIDTH_ACTUAL = 14.5;
    public static double CHASSIS_TRACK_WIDTH_CALCULATED = 14.695; //14.644

    public static double CHASSIS_kV = 0.017745;
    public static double CHASSIS_kA = 0.00362;
    public static double CHASSIS_kStatic = 0;

    public static double CHASSIS_MAX_VEL = 46.5;
    public static double CHASSIS_MAX_ACCEL = 46.5;

    public static double CHASSIS_MAX_ANGULAR_VEL_DEG = 202.2258;
    public static double CHASSIS_MAX_ANGULAR_VEL_RAD = Math.toRadians(CHASSIS_MAX_ANGULAR_VEL_DEG);

    public static double CHASSIS_MAX_ANGULAR_ACCEL_DEG = 202.2258;
    public static double CHASSIS_MAX_ANGULAR_ACCEL_RAD = Math.toRadians(CHASSIS_MAX_ANGULAR_ACCEL_DEG);
    //========== End of Chassis Settings ==========

    //========== Start of Hub Settings ==========
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    //========== End of Hub Settings ==========

    //========== Start of Tracking Wheel Localization Settings ==========
    public static boolean TRACKING_WHEEL_LEFT_ENCODER_REVERSED = false;
    public static boolean TRACKING_WHEEL_RIGHT_ENCODER_REVERSED = true;
    public static boolean TRACKING_WHEEL_FRONT_ENCODER_REVERSED = true;

    public static double TRACKING_WHEEL_TICKS_PER_REV = 2048;
    public static double TRACKING_WHEEL_WHEEL_RADIUS = 0.945; // in
    public static double TRACKING_WHEEL_GEAR_RATIO = 1;

    public static double TRACKING_WHEEL_LATERAL_DISTANCE = 14.48494; // in 24.5
    public static double TRACKING_WHEEL_FORWARD_OFFSET = -7; // in

    public static double TRACKING_WHEEL_X_MULTIPLIER = 1.020178874388808;
    public static double TRACKING_WHEEL_Y_MULTIPLIER = 1.016753837906208;
    //========== End of Tracking Wheel Localization Settings ==========

    public static double toCm(double in) {
        return in / 2.54;
    }

    public static double toIn(double cm) {
        return cm * 2.54;
    }
}