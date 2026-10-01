package org.firstinspires.ftc.teamcode.pedro.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestBasicSigleMotor extends LinearOpMode {

    private DcMotor fl; // Frente Esquerdo (Front Left)
    private DcMotor bl; // Trás Esquerdo (Back Left)
    private DcMotor fr; // Frente Direito (Front Right)
    private DcMotor br; // Trás Direito (Back Right)


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

        if (gamepad1.a) {

        }


}

}
