package org.firstinspires.ftc.teamcode.pedro.opmodes.blue;

import static com.pedropathing.api.Paths.curve;
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

@Autonomous(name="AUTO_BLUE_TOP", group="Blue")
public class autoTestNataa extends OpMode {

    boolean shooterLigado = false;

    Shooter shooter;
    Intake intake;
    Servos servo;
    Turret turret;
    PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;
    Zones zones;
    Pose currentPoseLocal;
    List<LynxModule> allHubs;

    Pose startingPoint = poseFactory.of(86.06466512702079,134.1431469720298,0);
    Pose collect1 = poseFactory.of(135.29337962962962, 130.47106481481478, 90);
    Pose shoot2 = poseFactory.of(116.01981481481482, 20.932870370370395, 72);
    Pose collect2 = poseFactory.of(100.94675925925927, 15.032638888888886, 250);
    Pose shoot3 = poseFactory.of(114.01981481481482, 17.932870370370395, 70);
    Pose collect3 = poseFactory.of(129.0388888888889, 97.13055555555556, 0);
    Pose shoot4 = poseFactory.of(104.5462962962963, 125.4074074074074, 300);
    Pose park = poseFactory.of(132.12685185185188, 51.289675925925924, 270);


    public Path START_TO_COLLECT1() {
        return curve(startingPoint, poseFactory.of(135.94, 96.9, 0),collect1).linear(startingPoint, collect1);
    }
    public Path COLLECT1_TO_SHOOT2() {
        return line(collect1, shoot2).linear(collect1, shoot2);
    }
    public Path SHOOT2_TO_COLLECT2() {
        return line(shoot2, collect2).linear(shoot2, collect2);
    }
    public Path COLLECT2_TO_SHOOT3() {
        return line(collect2, shoot3).linear(collect2, shoot3);
    }
    public Path SHOOT3_TO_COLLECT3() {
        return curve(shoot3, poseFactory.of(111.42245370370371, 67.38194444444444, 70),collect3).linear(shoot3, collect3);
    }
    public Path COLLECT3_TO_SHOOT4() {
        return line(collect3, shoot4).linear(collect3, shoot4);
    }
    public Path SHOOT4_TO_PARK() {
        return line(shoot4, park).linear(shoot4, park);
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

    public void loop() {
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

    public void start() {
        shooterLigado = true;
        Scheduler.schedule(routine());
        shooter.setLock(true);

    }
    public Command routine() {
        return parallel(
                sequential(
                        waitMs(2300),
                        shooter.shootPP(),
                        parallel(
                                follow(follower, START_TO_COLLECT1()),
                                intake.intakeOnlyUse()
                        ),
                        waitMs(1500),
                        intake.intakestopPP(),
                        follow(follower, COLLECT1_TO_SHOOT2()),
                        shooter.shootPP(),
                        parallel(
                                follow(follower, SHOOT2_TO_COLLECT2()),
                                intake.intakePP()
                        ),
                        waitMs(1500),
                        intake.intakestopPP(),
                        follow(follower, COLLECT2_TO_SHOOT3()),
                        shooter.shootPP(),
                        parallel(
                                follow(follower, SHOOT3_TO_COLLECT3()),
                                intake.intakePP()
                        ),
                        waitMs(1500),
                        intake.intakestopPP(),
                        follow(follower, COLLECT3_TO_SHOOT4()),
                        shooter.shootPP(),
                        follow(follower, SHOOT4_TO_PARK()),
                        instant(() -> shooterLigado = false)
                ),
                turret.autoaim(follower::pose),
                instant(() -> shooter.setLock(true))
        );

    }
}
