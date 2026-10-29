package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedro.subsystems.ServoTaura;

@TeleOp(name="AGRO", group="Tests")
public class Test_ServoTaura extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "flow");
        Servo servo1 = hardwareMap.get(Servo.class, "flow1");


       ServoTaura leftServo = new ServoTaura(servo);
       ServoTaura rightServo = new ServoTaura(servo1);

       leftServo.setDirection(Servo.Direction.FORWARD );
       rightServo.setDirection(Servo.Direction.REVERSE);



        leftServo.setPosition(0.0);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()) {
            if(gamepad1.a) {
                leftServo.setPosition(0.47);
                rightServo.setPosition(0.46);
            } else {
                leftServo.setPosition(0.0);
                rightServo.setPosition(0.0);
            }

            telemetry.addData("Pos comandada", "%.3f", leftServo.getPosition());
            telemetry.addData("Pos lida", "%.3f", leftServo.getUniversalPosition());
            telemetry.addData("Pos graus absoluto", "%.3f", leftServo.getRawPositionInDegrees());
            telemetry.addData("Pos graus incremental", "%.3f", leftServo.getIncrementalPositionInDegrees());

            telemetry.update();
        }
    }

}