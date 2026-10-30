
package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Tower;
import org.firstinspires.ftc.teamcode.pedro.subsystems.TowerCLAUDE;

@TeleOp
public class TeleOp_BLUECLAUDE extends OpMode {
    Intake intake;
    DriveTrain driveTrain;
    TowerCLAUDE tower;
    Agropecuaria agropecuaria;

    Follower follower;

    private boolean lastBothSticks = false;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        // Bulk read: todas as leituras de motor de um loop vem numa conversa so com cada hub (loop mais rapido)
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        agropecuaria = new Agropecuaria(hardwareMap);
        agropecuaria.up();

        follower = Constants.create(hardwareMap);
        tower = new TowerCLAUDE(hardwareMap, telemetry);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);

        follower.setPose(new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270)));
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();

        // ---------- comandos da torre ----------
        if (gamepad1.right_bumper && !tower.isLocked()) {

            tower.rotate(TowerCLAUDE.Direction.RIGHT);

        } else if (gamepad1.left_bumper && !tower.isLocked()) {

            tower.rotate(TowerCLAUDE.Direction.LEFT);

        } else if (!tower.isTracking() && !tower.isLocked()) {
            tower.stop();
        }

        if (gamepad1.dpadRightWasPressed() && !tower.isLocked()) {

            if (!tower.isTracking()) {

                if (currentPose.y() >= 70.5) {
                    tower.track(TowerCLAUDE.TrackingPose.TOP_GOAL_BLUE);
                } else {
                    tower.track(TowerCLAUDE.TrackingPose.BOTTOM_GOAL_BLUE);
                }
            } else {
                tower.stop();
            }
        }

        // Trava/destrava apertando os DOIS analogicos juntos.
        // (antes: "leftStickButtonWasPressed() && rightStickButtonWasPressed()" exigia os dois no MESMO loop,
        //  e o if estava invertido: travada chamava lock(), destravada chamava unlock(), entao nunca fazia nada)
        boolean bothSticks = gamepad1.left_stick_button && gamepad1.right_stick_button;
        if (bothSticks && !lastBothSticks) {
            if (tower.isLocked()) {
                tower.unlock();
            } else {
                tower.lock();
            }
        }
        lastBothSticks = bothSticks;

        // A torre atualiza DEPOIS dos comandos, no mesmo loop (sem um loop de atraso)
        tower.update(currentPose);

        driveTrain.driveStick();

        telemetry.addData("Mirando na colmeia", tower.isOnTarget() ? "SIM" : "nao");
        telemetry.addData("Loop (ms)", "%.0f", loopTimer.milliseconds()); // ideal: abaixo de ~25 ms
        loopTimer.reset();
        telemetry.update();
    }


}
