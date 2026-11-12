package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterConstants.FLOW_DOWN;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterConstants.FLOW_UP;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterConstants.LOCKED_POS;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterConstants.UNLOCKED_POS;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.RampaPotencia;
import org.firstinspires.ftc.teamcode.pedro.math.VoltageCompensator;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterConstants;

public class Shooter {

    Intake intake;
    Servos servos;
    ServoTaura lock;
    CRServo flow;
    public DcMotorEx rs, ls;
    VoltageCompensator voltageController;
    private double potenciaUpdate;
    private double alvoTps = 0;
    private double medidoTps = 0;
    private boolean encoderOff = false, isOn = false;
    public boolean isShooting = false, isLocked = true, nectar = false;
    private final RampaPotencia rampa = new RampaPotencia(2.0, 2.0);  // 0→1 em 0,5 s
    //-- GnHk28 has left the Commits

    public Shooter(HardwareMap hardwareMap, Intake intake) {
        servos = new Servos(hardwareMap);
        this.intake = intake;

        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");
        Servo lockServo = hardwareMap.get(Servo.class, "trava");
        lock = new ServoTaura(lockServo);
        flow = hardwareMap.get(CRServo.class, "cremalheira");

        ls.setDirection(DcMotorEx.Direction.REVERSE );
        rs.setDirection(DcMotorEx.Direction.FORWARD);

        ls.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ls.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rs.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        voltageController = new VoltageCompensator(hardwareMap);
    }
    public boolean isOn() {
        return isOn;
    }

    public void setLock(boolean locked) {
        isLocked = locked;

        if (!isLocked) {
            lock.setPosition(LOCKED_POS);
        } else {
            lock.setPosition(UNLOCKED_POS);
        }

    }

    public void shootNectar(boolean var) {
        nectar = var;
        if (var) {
            Scheduler.schedule(
                    sequential(
                            instant(() -> flow.setPower(FLOW_UP)),
                            waitMs(1850),
                            instant(() -> flow.setPower(0))
                    )
            );

        } else {
            Scheduler.schedule(
                    sequential(
                            instant(() -> flow.setPower(FLOW_DOWN)),
                            waitMs(1850),
                            instant(() -> flow.setPower(0))
                    )
            );
        }


    }


    //metodo p shootar no autooooo
    public Command shootPP() {
        return sequential(
                instant(()-> setLock(false)),
                instant(() -> intake.in(IntakeConstants.intakeShooting)),
                waitMs(1233),
                instant(() -> intake.stop()),
                instant(() -> setLock(true))
        );
    }

    public Command shootPPMatesc() {
        return sequential(
                instant(()-> setLock(false)),
                instant(() -> intake.in(IntakeConstants.intakeShooting)),
                waitMs(913),
                instant(() -> intake.stop()),
                instant(() -> setLock(true))
        );
    }
    //metodo p shootar no telop

    public void stop() {
        rs.setPower(0);
        ls.setPower(0);
        setLock(true);

        isOn = false;
    }

    public void setAlvo(double tps) {
        alvoTps = tps;

        isShooting = tps > 0;

    }

    public void update() {
        medidoTps = readVelocity();
        double pedido;
        if (alvoTps <= 0) {
            pedido = 0;
        } else {
            double ff = ShooterConstants.kS + ShooterConstants.kV * alvoTps; //potencia q se pa vai ser
            double fb = ShooterConstants.kP * (alvoTps - medidoTps); //correção do erro usando o p proporcional
            double compTensao = Constants.VoltageConstants.NOMINAL_VOLTAGE / Math.max(getTensao(), 9.0); //ajuste de tudo pela bateriaa
            pedido = Range.clip((ff + fb) * compTensao, 0, 1);
        }

        potenciaUpdate = rampa.calcular(pedido);
        ls.setPower(potenciaUpdate);
        rs.setPower(potenciaUpdate);
    }

    public boolean atSpeed() {
        return alvoTps > 0 && Math.abs(alvoTps - medidoTps) < ShooterConstants.READY_TOL;
    }

    private double readVelocity() {
        double veloLeft = getVelEsq();
        double veloRight = getVelDir();

        double max = Math.max(veloLeft, veloRight);
        double min = Math.min(veloLeft, veloRight);

        encoderOff = max > 200 && (max - min) > 0.15 * max;

        return encoderOff ? max :(max + min) / 2.0;
    }

    public boolean isEncoderOff() {
        return encoderOff;
    }

    public double getMedidoTps() {
        return medidoTps;
    }


    //-------------------------metodos testes--------------------------------
    public void setPotenciaDireta(double p) {
        p = Range.clip(p, 0, 1);
        ls.setPower(p);
        rs.setPower(p);
    }

    public double getVelEsq() { return Math.abs(ls.getVelocity()); }
    public double getVelDir() { return Math.abs(rs.getVelocity()); }
    public double getTensao() { return voltageController.getVoltage(); }

    public void telemetriaTeste(Telemetry telemetry) { //isso aqui a gnt anota pors testes ok
        telemetry.addData("Vel esq", "%.0f", getVelEsq());
        telemetry.addData("Vel dir", "%.0f", getVelDir());
        telemetry.addData("Tensao", "%.2f", getTensao());
        telemetry.addData("alvoTPS: ", ShooterConstants.ALVO_TPS);
        telemetry.addData("potencia que ta indo pros motores: ", potenciaUpdate);
        telemetry.addLine("-------------------------------");
        telemetry.addData("Alvo", 2300);
        telemetry.addData("Medido", "%.0f", getMedidoTps());
        telemetry.addData("Pronto", atSpeed());
        telemetry.addData("Encoder off", isEncoderOff());
    }

}
