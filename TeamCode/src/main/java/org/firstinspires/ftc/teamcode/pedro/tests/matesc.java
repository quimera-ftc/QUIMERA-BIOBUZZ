package org.firstinspires.ftc.teamcode.pedro.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class matesc extends OpMode {

    // Nomes abreviados dos motores
    private DcMotor fl; // Frente Esquerdo (Front Left)
    private DcMotor bl; // Trás Esquerdo (Back Left)
    private DcMotor fr; // Frente Direito (Front Right)
    private DcMotor br; // Trás Direito (Back Right)

    ElapsedTime runtime;
    int wheel = 0;


    @Override
    public void init() {
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

        runtime = new ElapsedTime();
        runtime.reset();


    }
    public void loop(){
        runtime.reset();
        if (wheel == 0) {

            while (runtime.milliseconds() < 3000) {
                fl.setPower(1);
            }
            fl.setPower(0);
            wheel = 1;
        } else if (wheel == 1) {

            while (runtime.milliseconds() < 3000) {
                fr.setPower(1);
            }
            fr.setPower(0);
            wheel = 2;
        } else if (wheel == 2) {

            while (runtime.milliseconds() < 3000) {
                bl.setPower(1);
            }
            bl.setPower(0);
            wheel = 3;
        } else if (wheel == 3) {

            while (runtime.milliseconds() < 3000) {
                br.setPower(1);
            }
            br.setPower(0);
            wheel = 4;
        }

    }

}

