package org.firstinspires.ftc.teamcode.pedro.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.VoltageCompensator;
import org.firstinspires.ftc.teamcode.pedro.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.math.RampaPotencia;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterCoefficients;

@TeleOp(name = "BASE TELEOP")


public class TestTeleopBase extends OpMode {
    DriveTrain driveTrain;
    Shooter shooter;
    Intake intake;
    Follower follower;
    List<LynxModule> allHubs;
    VoltageCompensator voltageCompensator;
    DcMotorEx backLeft, backRight, frontLeft, frontRight;
    RampaPotencia rampaPotencia;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.create(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        follower.setPose(PoseStorage.currentPose);
        voltageCompensator = new VoltageCompensator(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        rampaPotencia = new RampaPotencia(5, 6);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();


        if (gamepad1.aWasPressed()) {
            if(shooter.isShooting) {
                intake.toggle(IntakeConstants.intakeShooting);
            } else {
                intake.toggle(IntakeConstants.intakeCollecting);
            }

        }

        if (gamepad1.right_trigger > 0.3) {
            shooter.setAlvo(ShooterCoefficients.ALVO_TPS);
        } else {
            shooter.setAlvo(0);
        }

        Pose currentPose = follower.pose();

        telemetry.addData("x", currentPose.x());
        telemetry.addData("y", currentPose.y());
        telemetry.addData("heading", currentPose.heading());
        telemetry.addData("on", intake.isOn());
        telemetry.addData("power intake", intake.intake.getPower());
        telemetry.addData("voltage", voltageCompensator.getVoltage());
        telemetry.addData("current intake", intake.intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current rs", shooter.rs.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current ls", shooter.ls.getCurrent(CurrentUnit.AMPS));


        shooter.update();
        intake.update();
        driveTrain.driveStick(telemetry);
        telemetry.update();
    }
}
