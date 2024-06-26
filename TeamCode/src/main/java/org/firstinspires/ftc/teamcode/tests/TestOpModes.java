package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.ScrappyConstants;

public final class TestOpModes {
    public static final String GROUP = "testing";
    public static final boolean DISABLED = ScrappyConstants.IS_COMPETITION;

    private TestOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);
        manager.register(metaForClass(SelfDriveTest.class), SelfDriveTest.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(SplineTestEx.class), SplineTestEx.class);
        manager.register(metaForClass(RobotAutoDriveToAprilTag.class), RobotAutoDriveToAprilTag.class);
        manager.register(metaForClass(ServoTest.class), ServoTest.class);
        manager.register(metaForClass(PropDetectionTest.class), PropDetectionTest.class);
        manager.register(metaForClass(REV2mRealignmentTest.class), REV2mRealignmentTest.class);
        manager.register(metaForClass(SensorIMUNonOrthogonal.class), SensorIMUNonOrthogonal.class);
        manager.register(metaForClass(VisionTest.class), VisionTest.class);
    }
}
