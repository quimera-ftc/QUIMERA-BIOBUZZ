
package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Tower;

@TeleOp(name="TELEOP_BLUE_TOP", group="Blue")
public class teleop_BLUE_TOP extends OpMode {
    Intake intake;
    Agropecuaria agro;
    DriveTrain driveTrain;
    Tower tower;
    boolean flower = false;
    Pose startPose = new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270));
    Follower follower;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        tower = new Tower(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        agro = new Agropecuaria(hardwareMap);
        agro.up();
        follower.setPose(startPose);


        tower.track(Tower.TrackingPose.TOP_GOAL_BLUE);

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();
        tower.update(currentPose);


        if (gamepad1.right_bumper && !tower.isTracking()) {
            tower.rotate(Tower.Direction.RIGHT);
        } else if (gamepad1.left_bumper && !tower.isTracking()) tower.rotate(Tower.Direction.LEFT);

        if (gamepad1.dpadRightWasPressed()) {

            if (!tower.isTracking()) {

                if (currentPose.y() >= 70.75) {
                    tower.track(Tower.TrackingPose.TOP_GOAL_BLUE);
                } else tower.track(Tower.TrackingPose.BOTTOM_GOAL_BLUE);
            } else stop();
        }

        if (gamepad1.leftTriggerWasPressed()) {

            if (!flower) {
                agro.down();
            } else agro.up();

            flower = !flower;
        }


        if (gamepad1.xWasPressed()) {

            if (tower.isLocked()) {
                tower.lock();
            } else tower.unlock();
        }

        driveTrain.driveStick();
        telemetry.update();
    }


}