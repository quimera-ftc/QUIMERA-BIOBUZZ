package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    //right = 1 = bolinha nao passa
    //left = 0 = bolinha passa nao colocar como 0 pra nenhum robo bater e quebrar
    Servo servoLocker;
    Servo flowerRight;
    ServoTaura trava;
    Servo t1;

    public final double LOCKING = 1, DOWN_RIGHT = 0.75;
    public final double UP_RIGHT = 0;
    public final double NOT_LOCKING = 0.60;


    public Servos(@NonNull HardwareMap hardwareMap) {
        servoLocker = hardwareMap.get(Servo.class, "locker");
        flowerRight = hardwareMap.get(Servo.class, "flowerRight");
        t1 = hardwareMap.get(Servo.class, "trava");

        t1.setDirection(Servo.Direction.FORWARD);
        flowerRight.setDirection(Servo.Direction.FORWARD);

        trava = new ServoTaura(t1);


    }


    public void locking() {
        servoLocker.setPosition(LOCKING);
    }

    public void notLocking() {
        servoLocker.setPosition(NOT_LOCKING);
    }

    public double getPosition() {
        return servoLocker.getPosition();
    }

    public void agroUp() {
        flowerRight.setPosition(UP_RIGHT);
    }

    public void agroDown() {
        flowerRight.setPosition(DOWN_RIGHT);

    }
    public void open(){
        trava.setPosition(0);
    }
    public void close(){
        trava.setPosition(1);
    }

}
