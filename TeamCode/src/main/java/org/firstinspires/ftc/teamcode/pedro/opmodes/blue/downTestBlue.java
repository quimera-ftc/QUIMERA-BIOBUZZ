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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Servos;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Zones;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterCoefficients;

import java.util.List;

@Autonomous
public class downTestBlue extends OpMode {

    boolean shooterLigado = false;
    Shooter shooter;
    Intake intake;
    Turret turret;
    Servos agro;

    Zones zones;
    Pose currentPoseLocal;
    List<LynxModule> allHubs;

    PoseFactory poseFactory =  PoseFactory.degrees();



    private Follower follower;

    private final Pose torre = poseFactory.of(78.5051, 8.889, 90);

    private final Pose start =  poseFactory.of(91.0176, 37.9776, 270);

//    private final Pose collectStart = poseFactory.of(93.86, 16.61, 270);
    private  final Pose collect = poseFactory.of(94.9311,20.5272,270);

//    private final  Pose turneStart = poseFactory .of(94.04,26.23,270);

//    private final Pose turne = poseFactory.of(91.0176,37.9776,270);

//    private final Pose shootStart = poseFactory.of(93.8607, 16.6075, 270);
   private final Pose  shoot = poseFactory.of(82.0592,17.5201,90);

//    private final Pose fimStart = poseFactory.of(78.6305, 9.7349, 90);
   private final Pose fim = poseFactory.of(128.380745,20.8196,90);

    public Path START(){
        return line(torre,start).linear(torre,start);
    }
    public Path COLLECT(){
        return line(start,collect).linear(start,collect);
    }

    public Path SHOOT() {
        return line(collect, shoot).linear(collect, shoot);
    }
    public Path FIM(){
        return line(shoot,fim).linear(shoot, fim);
    }



    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(torre);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake);
        agro = new Servos(hardwareMap);
        turret = new Turret(hardwareMap, torre, Constants.Alliance.BLUE);

        zones = new Zones(ShooterCoefficients.ALVO_TPS)
                .addZone(50.96, 91.14, 105.14, 137.71, ShooterCoefficients.ALVO_TPS)
                .addZone(9.14, 50.96, 105.14, 137.71, ShooterCoefficients.ALVO_TPS_LADO)
                .addZone(91.14, 132.96, 105.14, 137.71, ShooterCoefficients.ALVO_TPS_LADO)

                .addZone(50.96, 91.14, 6.29, 38.86, ShooterCoefficients.ALVO_TPS)
                .addZone(9.14, 50.96, 6.29, 38.86, ShooterCoefficients.ALVO_TPS_LADO)
                .addZone(91.14, 132.96, 6.29, 38.86, ShooterCoefficients.ALVO_TPS_LADO);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void start() {
        shooterLigado = true;
        Scheduler.schedule(routine());
        }


    @Override
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

    public Command routine() {
        return parallel(
                sequential(
                        waitMs(5000),
                shooter.shootPP(),
                follow(follower, START()),
                parallel(
                        follow(follower, COLLECT()),
                        intake.intakePP()
                ),
                waitMs(1500),
                intake.intakestopPP(),

                follow(follower,SHOOT()),
                        shooter.shootPP(),
                        follow(follower, FIM()),
                        instant(() -> shooterLigado = false)

                ), turret.autoaim(follower::pose));
    }

}


