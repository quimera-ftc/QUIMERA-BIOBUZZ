package org.firstinspires.ftc.teamcode.pedro.opmodes.calibration;

import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.kD;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.kI;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.kP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.TurretController;
import org.firstinspires.ftc.teamcode.pedro.math.VoltageCompensator;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;

@TeleOp(name="CALIBRATE TURRET", group="Tests")
public class TestTurret extends OpMode {
    Agropecuaria agro;
    DriveTrain driveTrain;
    Turret turret;
    Pose startPose = new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270));
    Follower follower;
    VoltageCompensator voltageController;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.BLUE);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        agro = new Agropecuaria(hardwareMap);
        agro.up();
        follower.setPose(startPose);
        voltageController = new VoltageCompensator(hardwareMap);

        turret.resetEncoder();
        turret.setMode(Turret.Mode.IDLE);
    }

    @Override
    public void start() {
        turret.setMode(Turret.Mode.MANUAL);
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();
        turret.update(currentPose);


        if (gamepad1.right_bumper) {
            turret.setManualPower(0.7);
        } else if (gamepad1.left_bumper) {
            turret.setManualPower(-0.7);
        } else turret.setManualPower(0);

        if (gamepad1.dpadRightWasPressed()) {

            if (turret.getMode() == Turret.Mode.MANUAL) {
                turret.setMode(Turret.Mode.TRACKING);
            } else turret.setMode(Turret.Mode.MANUAL);

        }

        Pose turretPose = TurretController.getTurretPose(currentPose);
        double target = TurretController.computeTargetAngle(turretPose, TurretController.getTurretPose(turretPose));

        telemetry.addData("mode", turret.getMode());
        telemetry.addData("Turret X", turretPose.x());
        telemetry.addData("Turret Y", turretPose.y());
        telemetry.addData("target", target);
        telemetry.addData("deslocamento (Raw)", turret.getRawAngle());
        telemetry.addData("Erro Atual", turret.getDistanceToTarget());
        telemetry.addData("lime offset", turret.getLimeLightOffset());
        telemetry.addData("na mosca", turret.aligned() ? "sim" : "nao");
        telemetry.addData("tempo de ciclo (dt)", turret.getDt());
        telemetry.addData("voltage", "%.2f V", voltageController.getVoltage());
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);

        driveTrain.driveStick();
        telemetry.update();
    }
}
