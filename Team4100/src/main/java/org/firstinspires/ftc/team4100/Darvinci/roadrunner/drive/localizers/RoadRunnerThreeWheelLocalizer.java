package org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.util.Encoder;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class RoadRunnerThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = DarvinciSettings.TRACKING_WHEEL_TICKS_PER_REV;
    public static double WHEEL_RADIUS = DarvinciSettings.TRACKING_WHEEL_WHEEL_RADIUS; // in
    public static double GEAR_RATIO = DarvinciSettings.TRACKING_WHEEL_GEAR_RATIO; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = DarvinciSettings.TRACKING_WHEEL_LATERAL_DISTANCE; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = DarvinciSettings.TRACKING_WHEEL_FORWARD_OFFSET; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public static double X_MULTIPLIER = DarvinciSettings.TRACKING_WHEEL_X_MULTIPLIER; // Multiplier in the X direction
    public static double Y_MULTIPLIER = DarvinciSettings.TRACKING_WHEEL_Y_MULTIPLIER; // Multiplier in the Y direction

    private List<Integer> lastEncPositions, lastEncVels;

    public RoadRunnerThreeWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RB"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RF"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));

        leftEncoder.setDirection(DarvinciSettings.TRACKING_WHEEL_LEFT_ENCODER_REVERSED ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        rightEncoder.setDirection(DarvinciSettings.TRACKING_WHEEL_RIGHT_ENCODER_REVERSED ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        frontEncoder.setDirection(DarvinciSettings.TRACKING_WHEEL_FRONT_ENCODER_REVERSED ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULTIPLIER,
                encoderTicksToInches(rightPos) * X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }
}
