package org.firstinspires.ftc.team4100.Darvinci.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.team4100.Darvinci.DarvinciCore;
import org.firstinspires.ftc.team4100.Darvinci.DarvinciTeleOpBase;
import org.firstinspires.ftc.team4100.Darvinci.commands.BulkCacheHandler;

public class DarvinciTeleOp extends DarvinciTeleOpBase {
    private GamepadEx m_driverOne, m_driverTwo;
    private Button m_slideDown, m_slideUp, m_slideInc, m_slideDec;
    private int m_slideTopPos;
    private boolean m_driverToggle = false;

    public DarvinciTeleOp(DarvinciCore.AllianceType allianceType) {
        super(allianceType);
    }

    @Override
    public void initTeleOp() {
        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize gamepads
        m_driverOne = new GamepadEx(gamepad1);
        m_driverTwo = new GamepadEx(gamepad2);

        // Initialize slide top position tracker
        m_slideTopPos = robot.m_slide.getPosition();

        // Driver One
        m_driverOne.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand());
        m_driverOne.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand());
        m_driverOne.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> robot.m_slide.setRelPosition(0)));
        m_driverOne.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> robot.m_slide.setPosition(m_slideTopPos)));

        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> {
                    if (!m_driverToggle) {
                        robot.m_bucket.setRelPosition(-0.05);
                    } else {
                        robot.m_hook.in();
                    }
                }));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {
                    if (!m_driverToggle) {
                        robot.m_bucket.setRelPosition(0.05);
                    } else {
                        robot.m_hook.out();
                    }
                }));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (!m_driverToggle) {
                        robot.m_slide.setRelPosition(-100);
                    } else {
                        robot.m_elevator.lower();
                    }
                }));
        m_driverOne.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (!m_driverToggle) {
                        if (robot.m_slide.getTargetPosition() > m_slideTopPos) {
                            m_slideTopPos = robot.m_slide.getTargetPosition();
                        }
                        robot.m_slide.setRelPosition(100);
                    } else {
                        robot.m_elevator.rise();
                    }
                }));

        m_driverOne.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(robot.m_push::suck))
                .whenReleased(new InstantCommand(robot.m_push::stop));
        m_driverOne.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.m_push::spit))
                .whenReleased(new InstantCommand(robot.m_push::stop));
        new Trigger(() -> m_driverOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8)
                .whenActive(new InstantCommand(robot.m_intake::spit));
        new Trigger(() -> m_driverOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8)
                .whenActive(new InstantCommand(robot.m_intake::suck));

        m_driverOne.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> m_driverToggle = !m_driverToggle));

        // Initialize field-centric driving
//        register(robot.m_drive);
//        m_drive.setDefaultCommand(new InstantCommand(() -> {
//            m_drive.driveFieldCentric(m_driverOne.getLeftX(), m_driverOne.getLeftY(), m_driverOne.getRightX());
//        }));
    }

    @Override
    public void run() {
        telemetry.addData("hey!", ":)");
        telemetry.update();
    }
}