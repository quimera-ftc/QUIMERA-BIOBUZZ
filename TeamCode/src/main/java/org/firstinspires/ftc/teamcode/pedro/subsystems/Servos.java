package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Servos {
    //right = 1 = bolinha nao passa
    //left = 0 = bolinha passa nao colocar como 0 pra nenhum robo bater e quebrar

    Servo flowerRight;
    ServoTaura trava,agro;
    Servo t1, t2;
    private CRServo flow;
    public boolean mixed = false;
    public boolean sas = false;
    public final double DOWN_RIGHT = 0.80;
    public final double UP_RIGHT = 0;
    public final double CONST_UP = 1;
    public final double CONST_DOWN = -1;
    public final double TRAVA_OPEN = 1;
    public final double TRAVA_CLOSE = 0;


    public Servos(@NonNull HardwareMap hardwareMap) {
        flowerRight = hardwareMap.get(Servo.class, "flowerRight");
        t1 = hardwareMap.get(Servo.class, "trava");
        flow = hardwareMap.get(CRServo.class,"cremalheira");
        t2 = hardwareMap.get(Servo.class,"agro");



        t1.setDirection(Servo.Direction.FORWARD);
        flowerRight.setDirection(Servo.Direction.REVERSE);



        trava = new ServoTaura(t1);
        agro = new ServoTaura(t2);

    }

    public void contup(){
        flow.setPower(CONST_UP);
        mixed = true;
        sas = true;
    }
    public void contstop() {
        flow.setPower(0);
    }
    public void contdown(){
        flow.setPower(CONST_DOWN);
        mixed = false;
        sas = false;
    }


}
