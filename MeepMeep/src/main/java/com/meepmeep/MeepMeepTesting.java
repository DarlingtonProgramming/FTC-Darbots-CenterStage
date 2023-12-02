package com.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Robot dimensions
        final double robotWidth = 15.2;
        final double robotHeight = 17;

        // Drive constants
        final double maxVel = 64.3;
        final double maxAccel = 64.3;
        final double maxAngularVel = 4.836261028474933;
        final double maxAngularAccel = 4.836261028474933;
        final double trackWidth = 13.15;

        // Constraints
        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(Math.toRadians(30)),
                new MecanumVelocityConstraint(20, trackWidth)
        ));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(20);

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(robotWidth, robotHeight)
                .setConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel, trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -61.75, Math.toRadians(90)))
                                .waitSeconds(2)
                                .lineTo(new Vector2d(-45.5, -16))
                                .lineToSplineHeading(new Pose2d(-39, -11, Math.toRadians(180)))
                                .lineTo(new Vector2d(34, -13.5))
                                .lineToSplineHeading(new Pose2d(52, -28.5, Math.toRadians(180)))
                                .build()
                        );

        Image img = null;
        try {
            img = ImageIO.read(new File("/Users/aenverga/Documents/FTC-Darbots-CenterStage/MeepMeepTesting/src/main/resources/field-2023-juice-dark.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep
                .setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}