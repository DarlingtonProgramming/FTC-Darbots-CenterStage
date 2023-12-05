package org.firstinspires.ftc.team4100.Scrappy.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyCore;
import org.firstinspires.ftc.team4100.Scrappy.ScrappyTeleOpBase;

public class ScrappyTeleOp extends ScrappyTeleOpBase {
    private GamepadEx m_driverOne, m_driverTwo;
    private double m_headingOffset = 0;
    private int m_slideTopPos = 0;
    private double m_speed = 1;

    public ScrappyTeleOp(ScrappyCore.AllianceType allianceType) {
        super(allianceType, ScrappyCore.AllianceSide.LEFT);
    }

    @Override
    public void initTeleOp() {
        // Initialize gamepads
        m_driverOne = new GamepadEx(gamepad1);
        m_driverTwo = new GamepadEx(gamepad2);

        // Initialize slide top position tracker
        m_slideTopPos = robot.m_lift.getPosition() + 1400;

        /* Driver One */

        // Reset Field Centric Heading
        m_driverOne.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> m_headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

        // Speed
        m_driverOne.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> m_speed = 0.3));
        m_driverOne.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> m_speed = 1));

        // Lift Commands
        m_driverOne.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(robot.m_lift::toInitial));
        m_driverOne.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> robot.m_lift.setPosition(m_slideTopPos)));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> robot.m_lift.setRelativePosition(-160)));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (robot.m_lift.getTargetPosition() > m_slideTopPos) {
                        m_slideTopPos = robot.m_lift.getTargetPosition();
                    }
                    robot.m_lift.setRelativePosition(160);
                }));

        // Manual IntakeExt
        m_driverOne.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.m_intake::down));

        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(robot.m_intake::up));

        // Dropper
        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.m_dropper::drop))
                .whenReleased(new InstantCommand(robot.m_dropper::back));

        // Conveyor + Intake
        new Trigger(() -> m_driverOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(new ParallelCommandGroup(
                        new InstantCommand(robot.m_intake::spit),
                        new InstantCommand(robot.m_conveyor::down)
                ))
                .whenInactive(new ConditionalCommand(
                        new InstantCommand(),
                        new ParallelCommandGroup(
                                new InstantCommand(robot.m_intake::stop),
                                new InstantCommand(robot.m_conveyor::stop)
                        ),
                        () -> robot.m_intake.isSucking()
                ));
        new Trigger(() -> m_driverOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new ParallelCommandGroup(
                        new InstantCommand(robot.m_intake::suck),
                        new InstantCommand(robot.m_conveyor::up)
                ))
                .whenInactive(new ConditionalCommand(
                        new InstantCommand(),
                        new ParallelCommandGroup(
                                new InstantCommand(robot.m_intake::stop),
                                new InstantCommand(robot.m_conveyor::stop)
                        ),
                        () -> robot.m_intake.isSpitting()
                ));

        m_driverTwo.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> robot.m_plane.setRelativePosition(-0.05)));

        m_driverTwo.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> robot.m_plane.setRelativePosition(0.05)));
    }
    @Override
    public void run() {
        super.run();

        Pose2d currentPose = robot.m_drive.getPoseEstimate();
        double gyroAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        this.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gyroAngle - m_headingOffset, m_speed);
        robot.m_drive.update();

        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (raw)", Math.toDegrees(gyroAngle));
        telemetry.addData("Heading (offset)", Math.toDegrees(gyroAngle - m_headingOffset));
        telemetry.addLine();
        telemetry.addData("Dropper", robot.m_dropper.getPosition());
        telemetry.addData("Lift", robot.m_lift.getPosition());
        telemetry.addData("IntakeExt", robot.m_intake.getExtPosition());
        telemetry.addData("Plane", robot.m_plane.getPosition());
        telemetry.update();
    }
}