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
    private CRServo continuo;
    public boolean mixed = false;

    public final double  DOWN_RIGHT = 0.80;
    public final double UP_RIGHT = 0.1;
    public final double CONST_UP = 1;
    public final double CONST_DOWN = -1;
    public final double TRAVA_OPEN = 1;
    public final double TRAVA_CLOSE = 0;


    public Servos(@NonNull HardwareMap hardwareMap) {
        flowerRight = hardwareMap.get(Servo.class, "flowerRight");
        t1 = hardwareMap.get(Servo.class, "trava");
        continuo = hardwareMap.get(CRServo.class,"cremalheira");
        t2 = hardwareMap.get(Servo.class,"agro");



        t1.setDirection(Servo.Direction.FORWARD);
        flowerRight.setDirection(Servo.Direction.REVERSE);



        trava = new ServoTaura(t1);
        agro = new ServoTaura(t2);

    }


    public void agroUp() {
        flowerRight.setPosition(UP_RIGHT);
        agro.setPosition(0.25);
    }

    public void agroDown() {
        flowerRight.setPosition(DOWN_RIGHT);
        agro.setPosition(1);
    }

    public void open(){trava.setPosition(TRAVA_OPEN);}

    public void close(){
        trava.setPosition(TRAVA_CLOSE);
    }
    public void contup(){
        continuo.setPower(CONST_UP); mixed = true;
    }
    public void contstop() {
        continuo.setPower(0);
    }
    public void contdown(){
        continuo.setPower(CONST_DOWN); mixed = false;
    }


}
