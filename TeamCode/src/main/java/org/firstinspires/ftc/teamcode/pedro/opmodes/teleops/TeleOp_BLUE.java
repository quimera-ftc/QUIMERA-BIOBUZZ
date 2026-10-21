package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Tower;

@TeleOp
public class TeleOp_BLUE extends OpMode {

    Intake intake;
    DriveTrain driveTrain;
    Tower tower;

    Follower follower = Constants.create(hardwareMap);

    @Override
    public void init() {
        tower = new Tower(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();

        tower.update(currentPose);

        if (gamepad1.right_bumper) {
            tower.rotate(Tower.Direction.RIGHT);
        } else if (gamepad1.left_bumper) {
            tower.rotate(Tower.Direction.LEFT);
        } else if (!tower.isTracking()) {
            tower.stop();
        }

        if (gamepad1.dpadRightWasPressed()) {

            if (!tower.isTracking()) {

                if (currentPose.y() >= 70.5) {
                    tower.track(Tower.TrackingPoses.TOP_GOAL_BLUE);
                } else {
                    tower.track(Tower.TrackingPoses.BOTTOM_GOAL_BLUE);
                }
            } else {
                tower.stop();
            }
        }

        driveTrain.driveStick();
    }


}
