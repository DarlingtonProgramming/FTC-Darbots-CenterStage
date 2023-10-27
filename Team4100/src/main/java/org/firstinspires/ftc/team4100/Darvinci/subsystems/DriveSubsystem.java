package org.firstinspires.ftc.team4100.Darvinci.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;

public class DriveSubsystem extends SubsystemBase {
    private final DarvinciChassis m_drive;
    private double m_speed = 0.8;

    public DriveSubsystem(HardwareMap hwMap, Pose2d poseEstimate) {
        m_drive = new DarvinciChassis(hwMap);
        m_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_drive.setPoseEstimate(poseEstimate);
    }
    public double getDriveSpeed() {
        return m_speed;
    }

    public void setDriveSpeed(double speed) {
        m_speed = speed;
    }
    public void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX) {
        Pose2d currentPose = m_drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -leftStickY,
                -leftStickX
        ).rotated(-currentPose.getHeading());

        m_drive.setWeightedDrivePower(
                new Pose2d(
                    input.getX(),
                    input.getY(),
                    -rightStickX
                )
        );
    }

    public void driveRegular(double leftStickX, double leftStickY, double rightStickX) {
        m_drive.setWeightedDrivePower(
                new Pose2d(
                        -leftStickY,
                        -leftStickX,
                        -rightStickX
                )
        );
    }
}
