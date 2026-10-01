package org.firstinspires.ftc.teamcode.pedro.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class testchassis extends LinearOpMode {

    // Nomes abreviados dos motores
    private DcMotor fl; // Frente Esquerdo (Front Left)
    private DcMotor bl; // Trás Esquerdo (Back Left)
    private DcMotor fr; // Frente Direito (Front Right)
    private DcMotor br; // Trás Direito (Back Right)

    // Constantes de conversão (ajuste conforme o motor e roda do seu robô)
    static final double COUNTS_PER_MOTOR_REV = 537.6; // Ex: GOBUILDA / REV
    static final double WHEEL_DIAMETER_CM = 9.6;       // Diâmetro da roda em cm
    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * Math.PI);

    // Constante aproximada para o giro de 180 graus (ajuste empiricamente na pista)
    static final double TICKS_POR_GRAU = 11.5;

    @Override
    public void runOpMode() {
        // 1. Mapeamento dos motores no Hardware Map usando as siglas
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

        // 2. Direção dos motores (inverta REVERSE/FORWARD se necessário)
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // 3. Resetar os encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Pronto");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Passo 1: Andar 234 cm para frente
            andarRobo(234, 0.5);

//            sleep(500); // Pausa de segurança
//
//            // Passo 2: Virar 180 graus para a esquerda
//            int ticksGiro180 = (int) (180 * TICKS_POR_GRAU);
//            virarRobo(ticksGiro180, 0.5);
//
//            sleep(500); // Pausa de segurança
//
//            // Passo 3: Andar mais 234 cm
//            andarRobo(234, 0.5);

            telemetry.addData("Status", "Concluído!");
            telemetry.update();
        }
    }

    // --- Método para andar em linha reta ---
    public void andarRobo(double distanciaCm, double velocidade) {
        int ticksAlvo = (int) (distanciaCm * COUNTS_PER_CM);

        fl.setTargetPosition(fl.getCurrentPosition() + ticksAlvo);
        bl.setTargetPosition(bl.getCurrentPosition() + ticksAlvo);
        fr.setTargetPosition(fr.getCurrentPosition() + ticksAlvo);
        br.setTargetPosition(br.getCurrentPosition() + ticksAlvo);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double pot = Math.abs(velocidade);
        fl.setPower(pot);
        bl.setPower(pot);
        fr.setPower(pot);
        br.setPower(pot);

        while (opModeIsActive() && (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy())) {
            // Aguarda o movimento acabar
        }

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Método para virar o robô ---
    public void virarRobo(int ticksGiro, double velocidade) {
        // Esquerda: lado esquerdo recua (-), lado direito avança (+)
        fl.setTargetPosition(fl.getCurrentPosition() - ticksGiro);
        bl.setTargetPosition(bl.getCurrentPosition() - ticksGiro);
        fr.setTargetPosition(fr.getCurrentPosition() + ticksGiro);
        br.setTargetPosition(br.getCurrentPosition() + ticksGiro);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double pot = Math.abs(velocidade);
        fl.setPower(pot);
        bl.setPower(pot);
        fr.setPower(pot);
        br.setPower(pot);

        while (opModeIsActive() && (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy())) {
            // Aguarda o giro terminar
        }

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}