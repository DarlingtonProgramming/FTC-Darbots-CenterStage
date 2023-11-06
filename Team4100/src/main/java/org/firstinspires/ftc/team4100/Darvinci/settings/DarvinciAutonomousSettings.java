package org.firstinspires.ftc.team4100.Darvinci.settings;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class DarvinciAutonomousSettings {
    public static Pose2d BLUE_LEFT_STARTING_POSE = new Pose2d(10.5, 62, Math.toRadians(90));
    public static Pose2d RED_LEFT_STARTING_POSE = new Pose2d(-36.5, -62, Math.toRadians(270));
    public static Pose2d RED_RIGHT_STARTING_POSE = new Pose2d(7.5, -62, Math.toRadians(270));
    public static int APRILTAG_AVGED_ATTEMPTS = 10;
    public static double APRILTAG_X_OFFSET = -2.3;
    public static double APRILTAG_Y_OFFSET = 2.2;
    public static double APRILTAG_X_ROBOT_OFFSET = -9.4;
    public static double APRILTAG_Y_ROBOT_OFFSET = 0;
    public static int SLIDE_DUMP_HEIGHT = 680;
    public static int SLIDE_ENSURE_PIXEL_FACTOR = 70;
}