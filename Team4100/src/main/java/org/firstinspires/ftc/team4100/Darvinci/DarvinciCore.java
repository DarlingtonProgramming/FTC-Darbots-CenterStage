package org.firstinspires.ftc.team4100.Darvinci;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team4100.Darvinci.commands.BulkCacheHandler;
import org.firstinspires.ftc.team4100.Darvinci.roadrunner.drive.DarvinciChassis;
import org.firstinspires.ftc.team4100.Darvinci.settings.DarvinciSettings;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.BucketSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.HookSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.PlaneSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.PushSubsystem;
import org.firstinspires.ftc.team4100.Darvinci.subsystems.SlideSubsystem;

public class DarvinciCore extends Robot {
    public enum AllianceType {
        BLUE, RED
    }

    public final AllianceType ALLIANCE_TYPE;

    // Subsystems
    public DarvinciChassis m_drive;
    public BucketSubsystem m_bucket;
    public IntakeSubsystem m_intake;
    public SlideSubsystem m_slide;
    public PushSubsystem m_push;
    public ElevatorSubsystem m_elevator;
    public HookSubsystem m_hook;
    public PlaneSubsystem m_plane;

    public DarvinciCore(HardwareMap hardwareMap, AllianceType allianceType) {
        ALLIANCE_TYPE = allianceType;

        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize subsystems
        m_drive = new DarvinciChassis(hardwareMap);
        m_bucket = new BucketSubsystem(hardwareMap, DarvinciSettings.BUCKET_NAME);
        m_intake = new IntakeSubsystem(hardwareMap, DarvinciSettings.INTAKE_NAME);
        m_slide = new SlideSubsystem(hardwareMap, DarvinciSettings.SLIDE_NAME);
        m_push = new PushSubsystem(hardwareMap, DarvinciSettings.PUSH_NAME);
        m_elevator = new ElevatorSubsystem(hardwareMap, DarvinciSettings.ELEVATOR_NAME);
        m_hook = new HookSubsystem(hardwareMap, DarvinciSettings.HOOK_NAME);
        m_plane = new PlaneSubsystem(hardwareMap, DarvinciSettings.PLANE_NAME);
    }
}