package org.firstinspires.ftc.teamcode.pedro.math;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RampaPotencia {
    private final double subidaPorSeg;   // 4.0 = de 0 a 1 em 0,25 s
    private final double descidaPorSeg;  // pode ser maior: parar não gera pico
    private double atual = 0;
    private boolean primeiraChamada = true;
    private final ElapsedTime timer = new ElapsedTime();

    public RampaPotencia(double subidaPorSeg, double descidaPorSeg) {
        this.subidaPorSeg = subidaPorSeg;
        this.descidaPorSeg = descidaPorSeg;
    }

    public double calcular(double alvo) {
        double dt;
        if (primeiraChamada) {
            dt = 0.02;
            primeiraChamada = false;
        } else {
            dt = Math.min(timer.seconds(), 0.1);
        }
        timer.reset();

        boolean inverteu = alvo != 0 && atual != 0 && Math.signum(alvo) != Math.signum(atual);
        boolean subindo  = inverteu || Math.abs(alvo) > Math.abs(atual);

        double passo = (subindo ? subidaPorSeg : descidaPorSeg) * dt;
        atual += Range.clip(alvo - atual, -passo, passo);
        return atual;
    }

    public void reset() {
        atual = 0;
        primeiraChamada = true;
    }

    public double getAtual() { return atual; }
}