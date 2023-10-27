package org.firstinspires.ftc.team4100.Darvinci;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.team4100.Darvinci.util.PoseStorage;

public abstract class DarvinciTeleOpBase extends CommandOpMode {
    private final DarvinciCore.AllianceType m_allianceType;
    protected DarvinciCore robot;

    public DarvinciTeleOpBase(DarvinciCore.AllianceType allianceType) {
        m_allianceType = allianceType;
    }

    @Override
    public void initialize() {
        robot = new DarvinciCore(hardwareMap, m_allianceType);
        robot.m_drive.setPoseEstimate(PoseStorage.currentPose);

        initTeleOp();
    }

    protected void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX) {
        Pose2d currentPose = robot.m_drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -leftStickY,
                -leftStickX
        ).rotated(-currentPose.getHeading());

        robot.m_drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightStickX
                )
        );
    }

    protected void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX, double gyroAngle) {
        Pose2d currentPose = robot.m_drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -leftStickY,
                -leftStickX
        ).rotated(-gyroAngle);

        robot.m_drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightStickX
                )
        );
    }

    protected void driveRobotCentric(double leftStickX, double leftStickY, double rightStickX) {
        robot.m_drive.setWeightedDrivePower(
                new Pose2d(
                        -leftStickY,
                        -leftStickX,
                        -rightStickX
                )
        );
    }

    public abstract void initTeleOp();
}
