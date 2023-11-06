package org.firstinspires.ftc.team4100.Darvinci.teleop;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;
import org.firstinspires.ftc.team4100.Darvinci.DarvinciTeleOpBase;
import org.firstinspires.ftc.team4100.Darvinci.commands.BulkCacheHandler;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;

public class DarvinciTeleOp extends DarvinciTeleOpBase {
    private GamepadEx m_driverOne, m_driverTwo;
    private IMU m_imu;
    private int m_slideTopPos;
    private double m_headingOffset;
    private double m_speed = 1;

    public DarvinciTeleOp(DarvinciCore.AllianceType allianceType, double headingOffset) {
        super(allianceType);
        m_headingOffset = headingOffset;
    }

    @Override
    public void initTeleOp() {
        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize IMU
        m_imu = hardwareMap.get(IMU.class, "imu");
        m_imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(DarvinciSettings.HUB_LOGO_FACING_DIR, DarvinciSettings.HUB_USB_FACING_DIR)));

        // Initialize gamepads
        m_driverOne = new GamepadEx(gamepad1);
        m_driverTwo = new GamepadEx(gamepad2);

        // Initialize slide top position tracker
        m_slideTopPos = robot.m_slide.getPosition() + 700;

        // Driver One
        m_driverOne.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> m_speed = 0.3));
        m_driverOne.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> m_speed = 1));
        m_driverOne.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> robot.m_bucket.inRel(-0.13)),
                        new WaitCommand(200),
                        new InstantCommand(() -> robot.m_slide.setInitialRel(75)),
                        new WaitUntilCommand(() -> robot.m_slide.getRelPosition() < 85),
                        new WaitCommand(100),
                        new InstantCommand(robot.m_bucket::in),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.m_slide.setInitialRel(0))
                ));
        m_driverOne.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ParallelCommandGroup(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new InstantCommand(robot.m_bucket::out)
                                ),
                                new InstantCommand(),
                                () -> robot.m_bucket.getPosition() > DarvinciSettings.BUCKET_POS_OUT
                        ),
                        new InstantCommand(() -> robot.m_slide.setPosition(m_slideTopPos))
                ));

        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> robot.m_bucket.setRelPosition(-0.05)));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> robot.m_bucket.setRelPosition(0.05)));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> robot.m_slide.setRelPosition(-100)));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (robot.m_slide.getTargetPosition() > m_slideTopPos) {
                        m_slideTopPos = robot.m_slide.getTargetPosition();
                    }
                    robot.m_slide.setRelPosition(100);
                }));

        m_driverOne.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(robot.m_push::suck))
                .whenReleased(new InstantCommand(robot.m_push::stop));
        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.m_push::spit))
                .whenReleased(new InstantCommand(robot.m_push::stop));

        new Trigger(() -> m_driverOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8)
                .whenActive(new InstantCommand(robot.m_intake::spit))
                .whenInactive(new ConditionalCommand(
                        new InstantCommand(),
                        new InstantCommand(robot.m_intake::stop),
                        () -> robot.m_intake.isSucking()
                ));
        new Trigger(() -> m_driverOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8)
                .whenActive(new InstantCommand(robot.m_intake::suck))
                .whenInactive(new ConditionalCommand(
                        new InstantCommand(),
                        new InstantCommand(robot.m_intake::stop),
                        () -> robot.m_intake.isSpitting()
                ));

        m_driverOne.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> m_headingOffset = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

        m_driverTwo.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(robot.m_elevator::rise));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(robot.m_elevator::lower));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(robot.m_hook::out));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(robot.m_hook::in));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(robot.m_plane::hold));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.m_plane::shoot));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> robot.m_plane.setRelPosition(-0.5)));
        m_driverTwo.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> robot.m_plane.setRelPosition(0.5)));
    }
    @Override
    public void run() {
        super.run();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double centricHeading = AngleUnit.normalizeRadians(botHeading - m_headingOffset);

        double rotX = x * Math.cos(-centricHeading) - y * Math.sin(-centricHeading);
        double rotY = x * Math.sin(-centricHeading) + y * Math.cos(-centricHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double LFPower = (rotY + rotX + rx) / denominator;
        double LBPower = (rotY - rotX + rx) / denominator;
        double RFPower = (rotY - rotX - rx) / denominator;
        double RBPower = (rotY + rotX - rx) / denominator;

        robot.m_drive.setMotorPowers(m_speed * LFPower, m_speed * LBPower, m_speed * RBPower, m_speed * RFPower);
        robot.m_drive.update();

        telemetry.addData("hey!", ":)");
        telemetry.addData("offset", m_headingOffset);
        telemetry.update();
    }
}