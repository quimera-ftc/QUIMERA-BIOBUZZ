package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    //right = 1 = bolinha nao passa
    //left = 0 = bolinha passa nao colocar como 0 pra nenhum robo bater e quebrar

    Servo flowerRight;
    ServoTaura trava;
    Servo t1;

    public final double  DOWN_RIGHT = 0.80;
    public final double UP_RIGHT = 0;


    public Servos(@NonNull HardwareMap hardwareMap) {
        flowerRight = hardwareMap.get(Servo.class, "flowerRight");
        t1 = hardwareMap.get(Servo.class, "trava");

        t1.setDirection(Servo.Direction.FORWARD);
        flowerRight.setDirection(Servo.Direction.FORWARD);

        trava = new ServoTaura(t1);


    }


    public void agroUp() {
        flowerRight.setPosition(UP_RIGHT);
    }

    public void agroDown() {
        flowerRight.setPosition(DOWN_RIGHT);

    }
    public void open(){
        trava.setPosition(1);
    }
    public void close(){
        trava.setPosition(0);
    }

}
