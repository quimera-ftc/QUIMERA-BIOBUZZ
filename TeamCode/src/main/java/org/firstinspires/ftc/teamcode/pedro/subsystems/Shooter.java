package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;

public class Shooter {

    Intake intake;

    DcMotorEx rs, ls;
    public double targetBangBang = 0.95;
    private double alvoTps = 0;
    private double medidoTps = 0;
    private boolean encoderOff = false;
    private final double feedForward = 0.2;
    public double maxpower = 0.86, minpower = 0;
    public double start, now;
    boolean isOn = false;
    private VoltageSensor bateria;

    public Telemetry telemetry;
    public ElapsedTime runtime = new ElapsedTime();
    //PIDFCoefficients controller=  new PIDFCoefficients(0.5,0,0,0.5);
    //-- GnHk28 has left the Commits

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");

        ls.setDirection(DcMotorSimple.Direction.REVERSE);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);

        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.telemetry = telemetry;

        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = new Intake(hardwareMap);
        bateria = hardwareMap.voltageSensor.iterator().next();
    }

    public Shooter(HardwareMap hardwareMap) {
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");

        ls.setDirection(DcMotorSimple.Direction.REVERSE);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);

        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = new Intake(hardwareMap);
    }
    public boolean isOn() {
        return isOn;
    }

    public void powerUp() {
        rs.setPower(maxpower);
        ls.setPower(maxpower);

        isOn = true;
    }

    //metodo p shootar no autooooo
    public Command shootPP() {
        return sequential(
//                instant(() -> powerUp()),
                instant(() -> intake.in()),
                waitMs(1000),
                instant(() -> {
                    intake.stop();
                })
        );
    }

    //metodo p shootar no telop

    public void stop() {
        rs.setPower(0);
        ls.setPower(0);
        isOn = false;
    }

    public void setAlvo(double tps) {
        alvoTps = tps;

    }

    public void update() {
        medidoTps = readVelocity();

        if (alvoTps <= 0) {
            ls.setPower(0);
            rs.setPower(0);
            return;
        }

        double ff = Constants.ShooterCoefficients.kS + Constants.ShooterCoefficients.kV * alvoTps; //potencia q se pa vai ser
        double fb = Constants.ShooterCoefficients.kP * (alvoTps - medidoTps); //correção do erro usando o p proporcional
        double compTensao = 12.0 / Math.max(getTensao(), 9.0); //ajuste de tudo pela bateriaa

        double potencia = Range.clip((ff + fb) * compTensao, 0, 1);
        ls.setPower(potencia);
        rs.setPower(potencia);
    }

    public boolean atSpeed() {
        return alvoTps > 0 && Math.abs(alvoTps - medidoTps) < Constants.ShooterCoefficients.READY_TOL;
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

    //-------------------------metodos testes--------------------------------
    public void setPotenciaDireta(double p) {
        p = Range.clip(p, 0, 1);
        ls.setPower(p);
        rs.setPower(p);
    }

    public double getVelEsq() { return Math.abs(ls.getVelocity()); }
    public double getVelDir() { return Math.abs(rs.getVelocity()); }
    public double getTensao() { return bateria.getVoltage(); }

    public void telemetriaTeste(Telemetry telemetry, double potencia) { //isso aqui a gnt anota pors testes ok
        telemetry.addData("Potencia", "%.1f", potencia);
        telemetry.addData("Vel esq", "%.0f", getVelEsq());
        telemetry.addData("Vel dir", "%.0f", getVelDir());
        telemetry.addData("Tensao", "%.2f", getTensao());
    }

}
