package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Servos {
    //right = 1 = bolinha nao passa
    //left = 0 = bolinha passa nao colocar como 0 pra nenhum robo bater e quebrar

    ServoTaura trava;
    Servo t1, flowerRight, flowerLeft;;
    private CRServo flow;
    ElapsedTime timer = new ElapsedTime();
    public boolean mixed = false;
    public boolean sas = false;
    public final double  DOWN_RIGHT = 0.80;
    public final double UP_RIGHT = 0.20;
    public final double  DOWN_LEFT = 0.20;
    public final double UP_LEFT = 0.80;
    public final double CONST_UP = 1;
    public final double CONST_DOWN = -1;
    public final double TRAVA_OPEN = 1;
    public final double TRAVA_CLOSE = 0;


    public Servos(@NonNull HardwareMap hardwareMap) {
        flowerRight = hardwareMap.get(Servo.class, "flowerRight");
        flowerLeft = hardwareMap.get(Servo.class, "flowerLeft");
        t1 = hardwareMap.get(Servo.class, "trava");
        flow = hardwareMap.get(CRServo.class,"cremalheira");

        t1.setDirection(Servo.Direction.FORWARD);
        flowerLeft.setDirection(Servo.Direction.REVERSE);

        trava = new ServoTaura(t1);

        timer.reset();
    }

    public void agroDown() {
        flowerRight.setPosition(UP_RIGHT);
        flowerLeft.setPosition(DOWN_LEFT);
    }

    public void agroUp() {
        flowerRight.setPosition(DOWN_RIGHT);
        flowerLeft.setPosition(UP_LEFT);

    }

    public void open(){
        trava.setPosition(TRAVA_OPEN);
    }

    public void close(){

        trava.setPosition(TRAVA_CLOSE);
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
