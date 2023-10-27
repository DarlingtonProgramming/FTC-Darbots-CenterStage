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
        final double robotWidth = 16.3;
        final double robotHeight = 17.5;

        // Drive constants
        final double maxVel = 46.5;
        final double maxAccel = 46.5;
        final double maxAngularVel = Math.toRadians(202.2258);
        final double maxAngularAccel = Math.toRadians(202.2258);
        final double trackWidth = 14.695;

        // Starting trajectories
        final Pose2d blueLeftStartTraj = new Pose2d(10.5, 62, Math.toRadians(90));
        final Pose2d blueRightStartTraj = new Pose2d(-33.5, 62, Math.toRadians(90));
        final Pose2d redLeftStartTraj = new Pose2d(-36.5, -62, Math.toRadians(270));
        final Pose2d redRightStartTraj = new Pose2d(7.5, -62, Math.toRadians(270));

        // Detection
        final String detectedSide = "right";

        // Constraints
        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(Math.toRadians(30)),
                new MecanumVelocityConstraint(20, trackWidth)
        ));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(20);

        // Init MeepMeep
        MeepMeep meepMeep = new MeepMeep(700);

                RoadRunnerBotEntity blueRightRightParking = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(robotWidth, robotHeight)
                .setConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel, trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60.13, -35.04, Math.toRadians(180.00)))
                                .splineToSplineHeading(new Pose2d(-36.00, -35.04, Math.toRadians(180.00)), Math.toRadians(0.45))
                                .splineToConstantHeading(new Vector2d(-48.06, -49.21), Math.toRadians(-76.65))
                                .splineToSplineHeading(new Pose2d(-39.64, -51.32, Math.toRadians(180.00)), Math.toRadians(268.88))
                                .splineTo(new Vector2d(-12.83, -51.32), Math.toRadians(49.40))
                                .splineTo(new Vector2d(-12.06, -36.77), Math.toRadians(94.70))
                                .splineTo(new Vector2d(-11.11, -23.17), Math.toRadians(77.37))
                                .splineTo(new Vector2d(-11.11, 1.72), Math.toRadians(106.47))
                                .splineTo(new Vector2d(-16.28, 22.60), Math.toRadians(111.12))
                                .splineTo(new Vector2d(-36.77, 38.68), Math.toRadians(90.00))
                                .splineTo(new Vector2d(-52.85, 43.09), Math.toRadians(174.96))
                                .splineTo(new Vector2d(-59.94, 59.55), Math.toRadians(90.00))
                                .build()
                        );

//        // Blue Left
//        RoadRunnerBotEntity blueLeftLeftParking = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeBlueDark())
//                .setDriveTrainType(DriveTrainType.MECANUM)
//                .setDimensions(robotWidth, robotHeight)
//                .setConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel, trackWidth)
//                .followTrajectorySequence(drive -> {
//                        if (detectedSide.equals("left")) {
//                            return drive.trajectorySequenceBuilder(blueLeftStartTraj)
//                                    .lineToSplineHeading(new Pose2d(19, 39, Math.toRadians(92)))
//                                    .forward(14)
//                                    .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(180)))
//                                    .waitSeconds(3)
//                                    .forward(5)
//                                    .lineToSplineHeading(new Pose2d(41, 59, Math.toRadians(180)))
//                                    .lineToSplineHeading(new Pose2d(57, 59, Math.toRadians(180)))
//                                    .build();
//                        } else if (detectedSide.equals("middle")) {
//                            return drive.trajectorySequenceBuilder(blueLeftStartTraj)
//                                    .lineTo(new Vector2d(11.5, 34))
//                                    .forward(10)
//                                    .lineToSplineHeading(new Pose2d(40, 33, Math.toRadians(180)))
//                                    .waitSeconds(3)
//                                    .forward(5)
//                                    .lineToSplineHeading(new Pose2d(41, 59, Math.toRadians(180)))
//                                    .lineToSplineHeading(new Pose2d(57, 59, Math.toRadians(180)))
//                                    .build();
//                        } else {
//                            return drive.trajectorySequenceBuilder(blueLeftStartTraj)
//                                    .lineTo(new Vector2d(13.5, 58))
//                                    .lineToSplineHeading(new Pose2d(10.5, 33, Math.toRadians(15)), velConstraint, accelConstraint)
//                                    .back(3)
//                                    .lineToSplineHeading(new Pose2d(40, 26, Math.toRadians(180)))
//                                    .waitSeconds(3)
//                                    .forward(5)
//                                    .lineToSplineHeading(new Pose2d(41, 59, Math.toRadians(180)))
//                                    .lineToSplineHeading(new Pose2d(57, 59, Math.toRadians(180)))
//                                    .build();
//                        }
//                    }
//                );
//
//        RoadRunnerBotEntity blueLeftRightParking = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeBlueDark())
//                .setDriveTrainType(DriveTrainType.MECANUM)
//                .setDimensions(robotWidth, robotHeight)
//                .setConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel, trackWidth)
//                .followTrajectorySequence(drive -> {
//                    if (detectedSide.equals("left")) {
//                        return drive.trajectorySequenceBuilder(blueLeftStartTraj)
//                                .lineToSplineHeading(new Pose2d(19, 39, Math.toRadians(92)))
//                                .forward(14)
//                                .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(45.5, 15, Math.toRadians(180)))
//                                .build();
//                    } else if (detectedSide.equals("middle")) {
//                        return drive.trajectorySequenceBuilder(blueLeftStartTraj)
//                                .lineTo(new Vector2d(11.5, 34))
//                                .forward(10)
//                                .lineToSplineHeading(new Pose2d(40, 33, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(45.5, 15, Math.toRadians(180)))
//                                .build();
//                    } else {
//                        return drive.trajectorySequenceBuilder(blueLeftStartTraj)
//                                .lineTo(new Vector2d(13.5, 58))
//                                .lineToSplineHeading(new Pose2d(10.5, 33, Math.toRadians(15)), velConstraint, accelConstraint)
//                                .back(3)
//                                .lineToSplineHeading(new Pose2d(40, 26, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(45.5, 15, Math.toRadians(180)))
//                                .build();
//                    }
//                    }
//                );
//
//        // Blue Right
//
//        // Red Right
//        RoadRunnerBotEntity redRightLeftParking = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeRedDark())
//                .setDriveTrainType(DriveTrainType.MECANUM)
//                .setDimensions(robotWidth, robotHeight)
//                .setConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel, trackWidth)
//                .followTrajectorySequence(drive -> {
//                    if (detectedSide.equals("left")) {
//                        return drive.trajectorySequenceBuilder(redRightStartTraj)
//                                .lineTo(new Vector2d(13.5, -58))
//                                .lineToSplineHeading(new Pose2d(10.5, -33, Math.toRadians(345)), velConstraint, accelConstraint)
//                                .back(4)
//                                .lineToSplineHeading(new Pose2d(40, -30, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(43, -11, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(180)))
//                                .build();
//                    } else if (detectedSide.equals("middle")) {
//                        return drive.trajectorySequenceBuilder(redRightStartTraj)
//                                .lineTo(new Vector2d(11.5, -34))
//                                .forward(10)
//                                .lineToSplineHeading(new Pose2d(40, -37, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(43, -11, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(180)))
//                                .build();
//                    } else {
//                        return drive.trajectorySequenceBuilder(redRightStartTraj)
//                                .lineToSplineHeading(new Pose2d(19.5, -40, Math.toRadians(268)))
//                                .forward(14)
//                                .lineToSplineHeading(new Pose2d(40, -42, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(43, -11, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(180)))
//                                .build();
//                    }
//                    }
//                );
//
//        RoadRunnerBotEntity redRightRightParking = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeRedDark())
//                .setDriveTrainType(DriveTrainType.MECANUM)
//                .setDimensions(robotWidth, robotHeight)
//                .setConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel, trackWidth)
//                .followTrajectorySequence(drive -> {
//                    if (detectedSide.equals("left")) {
//                        return drive.trajectorySequenceBuilder(redRightStartTraj)
//                                .lineTo(new Vector2d(13.5, -58))
//                                .lineToSplineHeading(new Pose2d(10.5, -33, Math.toRadians(345)), velConstraint, accelConstraint)
//                                .back(4)
//                                .lineToSplineHeading(new Pose2d(40, -30, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(41, -59, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(57, -59, Math.toRadians(180)))
//                                .build();
//                    } else if (detectedSide.equals("middle")) {
//                        return drive.trajectorySequenceBuilder(redRightStartTraj)
//                                .lineTo(new Vector2d(11.5, -34))
//                                .forward(10)
//                                .lineToSplineHeading(new Pose2d(40, -37, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(41, -59, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(57, -59, Math.toRadians(180)))
//                                .build();
//                    } else {
//                        return drive.trajectorySequenceBuilder(redRightStartTraj)
//                                .lineToSplineHeading(new Pose2d(19.5, -40, Math.toRadians(268)))
//                                .forward(14)
//                                .lineToSplineHeading(new Pose2d(40, -42, Math.toRadians(180)))
//                                .waitSeconds(3)
//                                .forward(5)
//                                .lineToSplineHeading(new Pose2d(41, -59, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(57, -59, Math.toRadians(180)))
//                                .build();
//                    }
//                    }
//                );
//
//        // Red Left
//        RoadRunnerBotEntity redLeftRightParking = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeRedDark())
//                .setDriveTrainType(DriveTrainType.MECANUM)
//                .setDimensions(robotWidth, robotHeight)
//                .setConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel, trackWidth)
//                .followTrajectorySequence(drive -> {
//                            if (detectedSide.equals("left")) {
//                                return drive.trajectorySequenceBuilder(redLeftStartTraj)
//                                        .lineTo(new Vector2d(-45, -40))
//                                        .lineToSplineHeading(new Pose2d(-35.5, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(20, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(40, -30, Math.toRadians(180)))
//                                        .waitSeconds(3)
//                                        .forward(5)
//                                        .lineToSplineHeading(new Pose2d(41, -59, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(57, -59, Math.toRadians(180)))
//                                        .build();
//                            } else if (detectedSide.equals("middle")) {
//                                return drive.trajectorySequenceBuilder(redLeftStartTraj)
//                                        .lineTo(new Vector2d(-36.5, -34))
//                                        .lineToSplineHeading(new Pose2d(-39, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(20, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(40, -37, Math.toRadians(180)))
//                                        .waitSeconds(3)
//                                        .forward(5)
//                                        .lineToSplineHeading(new Pose2d(41, -59, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(57, -59, Math.toRadians(180)))
//                                        .build();
//                            } else {
//                                return drive.trajectorySequenceBuilder(redLeftStartTraj)
//                                        .lineTo(new Vector2d(-42, -52.5))
//                                        .lineToSplineHeading(new Pose2d(-32.5, -37, Math.toRadians(220)), velConstraint, accelConstraint)
//                                        .lineToSplineHeading(new Pose2d(-41.5, -36.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(-39, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(20, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(40, -42, Math.toRadians(180)))
//                                        .waitSeconds(3)
//                                        .forward(5)
//                                        .lineToSplineHeading(new Pose2d(41, -59, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(57, -59, Math.toRadians(180)))
//                                        .build();
//                            }
//                        }
//                );
//
//        RoadRunnerBotEntity redLeftLeftParking = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeRedDark())
//                .setDriveTrainType(DriveTrainType.MECANUM)
//                .setDimensions(robotWidth, robotHeight)
//                .setConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel, trackWidth)
//                .followTrajectorySequence(drive -> {
//                            if (detectedSide.equals("left")) {
//                                return drive.trajectorySequenceBuilder(redLeftStartTraj)
//                                        .lineTo(new Vector2d(-45, -40))
//                                        .lineToSplineHeading(new Pose2d(-35.5, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(20, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(40, -30, Math.toRadians(180)))
//                                        .waitSeconds(3)
//                                        .forward(5)
//                                        .lineToSplineHeading(new Pose2d(43, -11, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(180)))
//                                        .build();
//                            } else if (detectedSide.equals("middle")) {
//                                return drive.trajectorySequenceBuilder(redLeftStartTraj)
//                                        .lineTo(new Vector2d(-36.5, -34))
//                                        .lineToSplineHeading(new Pose2d(-39, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(20, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(40, -37, Math.toRadians(180)))
//                                        .waitSeconds(3)
//                                        .forward(5)
//                                        .lineToSplineHeading(new Pose2d(43, -11, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(180)))
//                                        .build();
//                            } else {
//                                return drive.trajectorySequenceBuilder(redLeftStartTraj)
//                                        .lineTo(new Vector2d(-42, -52.5))
//                                        .lineToSplineHeading(new Pose2d(-32.5, -37, Math.toRadians(220)), velConstraint, accelConstraint)
//                                        .lineToSplineHeading(new Pose2d(-41.5, -36.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(-39, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(20, -58.5, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(40, -42, Math.toRadians(180)))
//                                        .waitSeconds(3)
//                                        .forward(5)
//                                        .lineToSplineHeading(new Pose2d(43, -11, Math.toRadians(180)))
//                                        .lineToSplineHeading(new Pose2d(57, -11, Math.toRadians(180)))
//                                        .build();
//                            }
//                        }
//                );

//        Image img = null;
//        try {
//            img = ImageIO.read(new File("../../../../../resources/field-2023-juice-dark.png"));
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

        meepMeep//.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(blueLeftLeftParking)
//                .addEntity(blueLeftRightParking)
                .addEntity(blueRightRightParking)
//                .addEntity(redLeftLeftParking)
//                .addEntity(redLeftRightParking)
//                .addEntity(redRightLeftParking)
//                .addEntity(redRightRightParking)
                .start();
    }
}