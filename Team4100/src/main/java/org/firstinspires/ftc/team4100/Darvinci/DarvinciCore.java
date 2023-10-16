package org.firstinspires.ftc.team4100.Darvinci;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

public class DarvinciCore {
    private LinearOpMode DarvinciOpMode;
    private FtcDashboard Dashboard;

    private List<LynxModule> Hubs;
    private DcMotorEx LF, LB, RF, RB, Intake, Slide, Elevator;
    private CRServo Push;
    private Servo Bucket, Hooker;

    private double speed = 1;

    public DarvinciCore(LinearOpMode opMode) {
        this.DarvinciOpMode = opMode;
    }

    public void init() {
        this.Dashboard = FtcDashboard.getInstance();
        this.Hubs = DarvinciOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : this.Hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.LF = DarvinciOpMode.hardwareMap.get(DcMotorEx.class, "LF");
        this.LB = DarvinciOpMode.hardwareMap.get(DcMotorEx.class, "LB");
        this.RF = DarvinciOpMode.hardwareMap.get(DcMotorEx.class, "RF");
        this.RB = DarvinciOpMode.hardwareMap.get(DcMotorEx.class, "RB");

        this.RB.setDirection(DcMotorSimple.Direction.REVERSE);
        this.RF.setDirection(DcMotorSimple.Direction.REVERSE);

        this.LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DarvinciOpMode.telemetry.addData("DarvinciCore", "Initialized!");
        DarvinciOpMode.telemetry.update();
    }

//    public void setRotatorPower(double power) {
//        this.Rotator.setPower(power);
//    }
//
//    public void setThrowerPower(double power) {
//        this.Thrower.setPower(power);
//    }
//
//    public void setThrowerPosition(double pos) {
//        this.PushAlpha.setPosition(pos);
//        this.PushBeta.setPosition(pos);
//    }
//
//    public double getThrowerPosition() {
//        return this.PushAlpha.getPosition();
//    }

    public void driveRobot(double drive, double strafe, double rotate) {
        double LFPower = Range.clip(this.speed * (drive + rotate - strafe), -1.0, 1.0);
        double LBPower = Range.clip(this.speed * (drive + rotate + strafe), -1.0, 1.0);
        double RFPower = Range.clip(this.speed * (drive - rotate + strafe), -1.0, 1.0);
        double RBPower = Range.clip(this.speed * (drive - rotate - strafe), -1.0, 1.0);

        this.setPower(LFPower, LBPower, RFPower, RBPower);
    }

    public void setPower(double LFPower, double LBPower, double RFPower, double RBPower) {
        this.LF.setPower(LFPower);
        this.LB.setPower(LBPower);
        this.RF.setPower(RFPower);
        this.RB.setPower(RBPower);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void getCoordsFromAprilTag() {

    }
}
