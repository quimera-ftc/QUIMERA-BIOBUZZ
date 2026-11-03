
package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;

@TeleOp(name="TELEOP_BLUE_TOP", group="Blue")
public class teleop_BLUE_TOP extends OpMode {
    Intake intake;
    Agropecuaria agro;
    DriveTrain driveTrain;
    Turret turret;
    boolean flower = false;
    Pose startPose = new Pose(132.12, 51.28, Math.toRadians(270));
    Follower follower;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.BLUE);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        agro = new Agropecuaria(hardwareMap);
        agro.up();
        follower.setPose(startPose);
    }

    @Override
    public void start() {

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

        driveTrain.driveStick();
        telemetry.update();
    }


}