package org.firstinspires.ftc.teamcode.pedro.math;

import com.acmerobotics.dashboard.config.Config;

/**
 * Velocidade do shooter (ticks/s) em função da distância da turret até o gol (polegadas).
 *
 * Entre dois pontos a velocidade é interpolada em linha reta; abaixo do primeiro ponto
 * vale o primeiro, acima do último vale o último.
 *
 * Os pontos são ajustáveis pelo FTC Dashboard sem recompilar. As distâncias precisam
 * estar em ordem crescente.
 *
 * Como calibrar: parar o robô a cada distância (ver "distancia gol" na telemetria),
 * disparar 5 artefatos, ajustar o TPS daquele ponto até acertar, anotar e passar
 * para o próximo. Os valores iniciais são todos iguais ao ALVO_TPS de hoje (1210),
 * então, antes da calibração, o robô se comporta igual ao código anterior.
 */
@Config
public class TabelaDisparo {
    public static double DIST_1 = 30.5,  TPS_1 = 1525;
    public static double DIST_2 = 50.6,  TPS_2 = 1225;
    public static double DIST_3 = 53.2,  TPS_3 = 1290;
    public static double DIST_4 = 60.6,  TPS_4 = 1360;

    private TabelaDisparo() {}

    public static double tpsParaDistancia(double distancia) {
        // lido a cada chamada para refletir mudanças feitas pelo Dashboard
        double[] dist = {DIST_1, DIST_2, DIST_3, DIST_4};
        double[] tps  = {TPS_1,  TPS_2,  TPS_3,  TPS_4};

        if (Double.isNaN(distancia) || distancia <= dist[0]) return tps[0];

        for (int i = 1; i < dist.length; i++) {
            if (distancia <= dist[i]) {
                double faixa = dist[i] - dist[i - 1];
                if (faixa <= 0) return tps[i];               // pontos fora de ordem: não divide por zero
                double t = (distancia - dist[i - 1]) / faixa;
                return tps[i - 1] + t * (tps[i] - tps[i - 1]);
            }
        }
        return tps[tps.length - 1];
    }
}