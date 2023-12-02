package org.firstinspires.ftc.team4100.Scrappy;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.team4100.Scrappy.util.PoseStorage;

public abstract class ScrappyTeleOpBase extends CommandOpMode {
    private final ScrappyCore.AllianceType m_allianceType;
    private final ScrappyCore.AllianceSide m_allianceSide;

    protected ScrappyCore robot;
    protected IMU imu;

    public ScrappyTeleOpBase(ScrappyCore.AllianceType allianceType, ScrappyCore.AllianceSide allianceSide) {
        m_allianceType = allianceType;
        m_allianceSide = allianceSide;
    }

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new ScrappyCore(hardwareMap, m_allianceType, m_allianceSide);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(xyzOrientation(180, 90, 10)));
        imu.initialize(parameters);

        robot.m_drive.setPoseEstimate(PoseStorage.currentPose);
        robot.m_lift.toInitial();
        robot.m_dropper.back();

        initTeleOp();
    }

    protected void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX, double speed) {
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
                ).times(speed)
        );
    }

    protected void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX, double gyroAngle, double speed) {
        Vector2d input = new Vector2d(
                -leftStickY,
                -leftStickX
        ).rotated(-gyroAngle);

        robot.m_drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightStickX
                ).times(speed)
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
