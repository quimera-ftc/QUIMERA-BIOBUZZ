package org.firstinspires.ftc.teamcode.pedro.opmodes.blue;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Servos;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Zones;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterConstants;

import java.util.List;
@Autonomous(name="AUTO_BLUE_TOP_ALLIANCE", group="Blue")
public class autoAllianceBlue extends OpMode {
    boolean shooterLigado = false;

    Shooter shooter;
    Intake intake;
    Servos servo;
    Turret turret;
    Zones zones;
    private Follower follower;
    Pose currentPoseLocal;
    List<LynxModule> allHubs;

    PoseFactory poseFactory = PoseFactory.degrees();

    Pose startingPoint = poseFactory.of(86.06466512702079, 134.1431469720298, 0);
    Pose collect1 = poseFactory.of(130.97, 131.97, 0);
    Pose shoot2 = poseFactory.of(104.9, 113.2, 300);
    Pose collect2 = poseFactory.of(130.53, 95.13, 0);
    Pose shoot3 = poseFactory.of(102.93, 111.97, 330);
    Pose park = poseFactory.of(132.12685185185188, 51.289675925925924, 270);

    Path START_TO_COLLECT1() {
        return line(startingPoint, collect1).linear(startingPoint, collect1);
    }
    Path COLLECT1_TO_SHOOT2() {
        return line(collect1, shoot2).linear(collect1, shoot2);
    }
    Path SHOOT2_TO_COLLECT2() {
        return line(shoot2, collect2).linear(shoot2, collect2);
    }
    Path COLLECT2_TO_SHOOT3() {
        return line(collect2, shoot3).linear(collect2, shoot3);
    }
    Path SHOOT3_TO_PARK() {
        return line(shoot3, park).linear(shoot3, park);
    }

    public void loop(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();
        Scheduler.execute();
        intake.update();

        currentPoseLocal = follower.pose();
        turret.update(currentPoseLocal);

        shooter.setAlvo(shooterLigado
                ? zones.getTPS(currentPoseLocal.x(), currentPoseLocal.y())
                : 0);
        shooter.update();

    }

    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPoint);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake);
        servo = new Servos(hardwareMap);
        turret = new Turret(hardwareMap, startingPoint, Constants.Alliance.BLUE);
        turret.resetEncoder();

        zones = new Zones(ShooterConstants.IDLE_TPS)
                .addZone(57.88,107.45,95.41,144.00, ShooterConstants.ALVO_TPS)
                .addZone(107.45,141.50,95.41,138.87, ShooterConstants.ALVO_TPS_LADO)
                .addZone(0.00, 57.88, 95.41, 144.00, ShooterConstants.ALVO_TPS_AWAY)

                .addZone(57.88, 107.45, 0.00, 48.59, ShooterConstants.ALVO_TPS)
                .addZone(107.45, 141.50, 5.13, 48.59, ShooterConstants.ALVO_TPS_LADO)
                .addZone(0.00, 57.88, 0.00, 48.59, ShooterConstants.ALVO_TPS_AWAY);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void start() {
        shooterLigado = true;
        Scheduler.schedule(routine());

    }

    public Command routine() {
        return parallel(
                sequential(
                        waitMs(2000),
                        shooter.shootPP(),
                        parallel(
                                follow(follower, START_TO_COLLECT1()),
                                intake.intakePP()
                                ),
                        waitMs(1500),
                        intake.intakestopPP(),
                        follow(follower, COLLECT1_TO_SHOOT2()),
                        waitMs(12000),
                        shooter.shootPP(),
                        parallel(
                                follow(follower, SHOOT2_TO_COLLECT2()),
                                intake.intakePP()
                        ),
                        waitMs(1500),
                        intake.intakestopPP(),
                        follow(follower, COLLECT2_TO_SHOOT3()),
                        shooter.shootPP(),
                        follow(follower, SHOOT3_TO_PARK()),
                        instant(() -> shooterLigado = false)
                ),
                turret.autoaim(follower::pose),
                instant(() -> shooter.setLock(true))

        );
    }


}
