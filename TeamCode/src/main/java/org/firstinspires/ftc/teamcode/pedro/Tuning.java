//package org.firstinspires.ftc.teamcode.pedro;
//
//import com.pedropathing.algorithm.Foresight;
//import com.pedropathing.revhub.drivetrains.Mecanum;
//import com.pedropathing.revhub.drivetrains.tuners.MecanumTuner;
//import com.pedropathing.revhub.localizers.PinpointLocalizer;
//import com.pedropathing.revhub.localizers.tuners.PinpointTuner;
//import com.pedropathing.algorithm.tuners.ForesightTuner;
//import com.pedropathing.tuning.Procedure;
//import com.pedropathing.tuning.Tests;
//import com.pedropathing.tuning.Tuner;
//
///**
// * Todas as rotinas de tuning do AutoTune ficam aqui.
// * Depois de buildar e enviar pro robô, acesse http://192.168.43.1:10158
// * enquanto estiver conectado ao Robot Controller para abrir o AutoTune.
// *
// * Ordem recomendada de tuning:
// *   1) mecanumTuner   -> Drivetrain
// *   2) pinpointTuner  -> Localizer
// *   3) foresightTuner -> Algoritmo (Foresight)
// *   4) tests          -> Verificação final
// */
//public class Tuning {
//
//    // 1) Drivetrain: descobre nomes/direções dos motores do Mecanum
//    @Tuner
//    public static Procedure mecanumTuner() {
//        return new MecanumTuner();
//    }
//
//    // 2) Localizer: descobre offsets/direções dos pods do Pinpoint
//    @Tuner
//    public static Procedure pinpointTuner() {
//        return new PinpointTuner();
//    }
//
//    // 3) Foresight: descobre velocidades, desacelerações e ganhos (kP)
//    //    Precisa do Drivetrain e Localizer já tunados nos passos anteriores.
//    @Tuner
//    public static Procedure foresightTuner() {
//        return new ForesightTuner(
//                (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
//                (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
//        );
//    }
//
//    // 4) Tests: verifica tudo junto (localização, direção, hold, linha, curva...)
//    @Tuner
//    public static Procedure tests() {
//        return new Tests(
//                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
//                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
//                () -> new Foresight(Constants.foresightConfig)
//        );
//    }
//}
