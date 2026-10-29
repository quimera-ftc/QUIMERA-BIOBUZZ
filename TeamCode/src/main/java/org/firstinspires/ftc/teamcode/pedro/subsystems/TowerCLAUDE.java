package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Torreta com controle proprio (PD + feedforward), no lugar do RUN_TO_POSITION do hub.
 *
 *   potencia = kP*erro + kD*(velAlvo - velTorre) + kV*velAlvo + kA*acelAlvo  (+ kS no sentido do movimento)
 *
 *   kP  empurra para o alvo (proporcional ao erro)
 *   kD  amortece: freia antes de chegar, para nao passar do alvo
 *   kV  feedforward: gira junto com o robo ANTES de aparecer erro (e o que deixa a mira "grudada")
 *   kS  potencia minima para vencer o atrito da torreta
 *   kA  (opcional) ajuda quando o robo arranca o giro
 *
 * Zona morta: com o alvo parado e erro menor que TOLERANCE, o motor desliga (o BRAKE segura).
 * So volta a corrigir quando o erro passa de RESUME. E isso que acaba com o "samba" perto do alvo.
 *
 * Os ganhos sao "public static" para o Test_TOWER poder mudar com o gamepad2 sem recompilar.
 */
public class TowerCLAUDE {

    // ===================== MECANICA =====================
    // HD Hex: 28 ticks por volta do motor. Cartucho "4:1" da UltraPlanetary e na real 76/21 (~3,62:1).
    // Engrenagem do motor 15 dentes -> coroa da torreta 50 dentes. Resultado: ~0,94 tick/grau (1 tick ~ 1,07 grau).
    public static final double TICKS_PER_DEGREE = 28.0 * (76.0 / 21.0) * (50.0 / 15.0) / 360.0;

    // Posicao da torreta em relacao ao centro do robo, em POLEGADAS (convencao do Pedro: x = frente, y = esquerda).
    // Antes estava "new double[] {0, 1,772}": o Java leu TRES numeros (0, 1 e 772), entao o offset usado era 1, nao 1,772.
    private static final double TURRET_OFFSET_FORWARD = 0.0;
    private static final double TURRET_OFFSET_LEFT = 1.772;

    // ===================== GANHOS (ajustar no robo com o Test_TOWER) =====================
    public static double kP = 0.02;     // potencia por grau de erro
    public static double kD = 0.003;    // potencia por grau/s de diferenca de velocidade
    public static double kV = 0.00035;  // potencia por grau/s de velocidade do alvo (~ 1 / velocidade maxima)
    public static double kS = 0.06;     // potencia minima para a torreta comecar a girar (MEDIR com o Test_TOWER)
    public static double kA = 0.0;      // potencia por grau/s^2 (opcional, comece em 0; ajuda torreta pesada a acompanhar arrancadas)
    public static double MAX_POWER = 1.0;

    // ===================== ZONA MORTA (anti-samba) =====================
    public static double TOLERANCE = 1.5;     // graus: abaixo disso a torreta "assenta" e o motor desliga
    public static double RESUME = 2.5;        // graus: assentada, so volta a corrigir acima disso
    public static double SETTLE_SPEED = 15;   // graus/s: so assenta se a torreta estiver quase parada
    public static double TARGET_MOVING = 8;   // graus/s: acima disso o alvo esta se mexendo (robo andando/girando)
    public static double AIM_TOLERANCE = 3;   // graus: isOnTarget() fica true abaixo disso

    // ===================== OUTROS =====================
    public static double MAX_SPIN = 150;           // limite de giro para cada lado (graus); deixe ~15 graus de folga ate o limite fisico (cabos)
    public static double MANUAL_SPEED = 200;       // graus/s com o bumper apertado
    public static double MANUAL_ACCEL = 800;       // graus/s^2: rampa do giro manual (e freada antes do limite)
    public static double IDLE_BRAKE = 0.006;       // potencia por grau/s para frear ao parar (stop())
    public static double IDLE_BRAKE_SPEED = 40;    // graus/s: abaixo disso o BRAKE do motor termina de parar
    public static boolean FEEDFORWARD = true;      // false = so posicao, como era antes (bom para comparar)
    public static double VELOCITY_SMOOTHING = 0.5; // 0..1: maior responde mais rapido, porem com mais ruido
    public static final double NOMINAL_VOLTAGE = 12.0;

    private enum State {
        IDLE,       // para (freia se estiver girando) e deixa o motor desligado
        TRACKING,   // mirando na colmeia
        MANUAL,     // girando com os bumpers
        HOLD,       // parado num angulo fixo (goTo)
        LOCKED      // travado onde estava
    }

    public enum TrackingPose {
        TOP_GOAL_BLUE,
        BOTTOM_GOAL_BLUE,
        TOP_GOAL_RED,
        BOTTOM_GOAL_RED
    }

    public enum Direction {
        RIGHT,
        LEFT
    }

    private final Pose topNestBLUE = new Pose(84, 84);
    private final Pose bottomNestBLUE = new Pose(84, 59);
    private final Pose topNestRED = new Pose(60, 84);
    private final Pose bottomNestRED = new Pose(60, 59);

    private final DcMotorEx motor;
    private final Telemetry telemetry;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime clock = new ElapsedTime();

    private State state = State.IDLE, stateBeforeLock = State.IDLE;
    private Pose goal = topNestBLUE;

    // leituras do loop atual (graus e graus/s)
    private double angle, velocity;
    // alvo
    private double targetAngle, targetVelocity, targetAccel;
    private boolean targetAtLimit, settled;
    private double error, power, lastSentPower = Double.NaN;
    private int manualDirection;
    private double manualVelocity;

    // velocidade do robo, estimada pela diferenca entre poses
    private boolean hasLastPose = false;
    private double lastX, lastY, lastHeading, lastTime;
    private double robotVx, robotVy, robotOmega, robotAlpha; // pol/s, pol/s, graus/s, graus/s^2

    private double voltage = NOMINAL_VOLTAGE, lastVoltageRead = -1;

    public TowerCLAUDE(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, true);
    }

    /**
     * @param resetEncoder false = mantem o zero de antes (ex.: vindo do autonomo, sem desligar o robo).
     *                     true  = a posicao atual vira o 0 (centralize a torreta ANTES do init!).
     */
    public TowerCLAUDE(HardwareMap hardwareMap, Telemetry telemetry, boolean resetEncoder) {
        this.telemetry = telemetry;
        motor = hardwareMap.get(DcMotorEx.class, "tower");
        motor.setDirection(DcMotor.Direction.REVERSE);
        if (resetEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // O controle agora e nosso: o hub so aplica a potencia que mandarmos.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setPower(0);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    // ===================== COMANDOS =====================

    /** Gira na mao enquanto for chamado a cada loop. 1 = esquerda (anti-horario), -1 = direita (horario). */
    public void rotate(int direction) {
        if (state == State.LOCKED) {
            return;
        }
        if (state != State.MANUAL) {
            state = State.MANUAL;
            targetAngle = angle; // comeca de onde esta
            manualVelocity = 0;
            settled = false;
        }
        manualDirection = (int) Math.signum(direction);
    }

    // LEFT = anti-horario, RIGHT = horario
    public void rotate(@NonNull Direction direction) {
        rotate(direction == Direction.LEFT ? 1 : -1);
    }

    public void track(@NonNull TrackingPose pose) {
        switch (pose) {
            case TOP_GOAL_BLUE:
                goal = topNestBLUE;
                break;
            case BOTTOM_GOAL_BLUE:
                goal = bottomNestBLUE;
                break;
            case TOP_GOAL_RED:
                goal = topNestRED;
                break;
            case BOTTOM_GOAL_RED:
                goal = bottomNestRED;
                break;
        }

        if (state == State.LOCKED) {
            return;
        }
        if (state != State.TRACKING) {
            settled = false;
        }
        state = State.TRACKING;
    }

    /** Vai para um angulo fixo (graus, 0 = frente do robo) e segura la. Util no autonomo e nos testes. */
    public void goTo(double degrees) {
        if (state == State.LOCKED) {
            return;
        }
        state = State.HOLD;
        targetAngle = Range.clip(degrees, -MAX_SPIN, MAX_SPIN);
        settled = false;
    }

    public void lock() {
        if (state == State.LOCKED) {
            return;
        }
        stateBeforeLock = state == State.TRACKING ? State.TRACKING : State.IDLE;
        state = State.LOCKED;
        targetAngle = angle;
        settled = false;
    }

    public void unlock() {
        if (state != State.LOCKED) {
            return;
        }
        state = stateBeforeLock;
        settled = false;
    }

    public void stop() {
        if (state == State.IDLE) {
            return; // ja parada (o TeleOp chama isso todo loop)
        }
        state = State.IDLE;
        settled = false;
        manualDirection = 0;
        manualVelocity = 0;
        setPower(0);
    }

    // ===================== LOOP =====================

    /** Chame UMA vez por loop, depois de follower.update() e dos comandos do gamepad. */
    public void update(@NonNull Pose pose) {
        double now = clock.seconds();
        // se ficou muito tempo sem update() (ex.: Test_TOWER medindo kS), recomeca a estimativa de velocidade
        if (hasLastPose && now - lastTime > 0.25) {
            hasLastPose = false;
            robotVx = robotVy = robotOmega = robotAlpha = 0;
        }
        double dt = hasLastPose ? Range.clip(now - lastTime, 0.001, 0.1) : 0.02;

        // 1) leituras (uma vez so por loop; com bulk read vem tudo numa conversa com o hub)
        angle = motor.getCurrentPosition() / TICKS_PER_DEGREE;
        velocity = motor.getVelocity() / TICKS_PER_DEGREE;
        if (lastVoltageRead < 0 || now - lastVoltageRead > 0.5) { // leitura lenta: 2x por segundo basta
            voltage = voltageSensor.getVoltage();
            lastVoltageRead = now;
        }

        // 2) velocidade do robo (para o feedforward)
        updateRobotVelocity(pose, dt);
        lastTime = now;

        // 3) alvo
        targetAtLimit = false;
        switch (state) {
            case TRACKING:
                computeTrackingTarget(pose);
                break;
            case MANUAL: {
                // o alvo acelera e freia aos poucos (MANUAL_ACCEL), para a torreta conseguir acompanhar
                double previousVelocity = manualVelocity;
                double step = MANUAL_ACCEL * dt;
                manualVelocity += Range.clip(manualDirection * MANUAL_SPEED - manualVelocity, -step, step);
                // perto do limite, so deixa a velocidade que ainda da para frear antes de chegar nele
                double dir = Math.signum(manualVelocity);
                if (dir != 0) {
                    double room = Math.max(MAX_SPIN - targetAngle * dir, 0);
                    manualVelocity = dir * Math.min(Math.abs(manualVelocity), Math.sqrt(2 * MANUAL_ACCEL * room));
                }
                targetAngle = Range.clip(targetAngle + manualVelocity * dt, -MAX_SPIN, MAX_SPIN);
                targetVelocity = manualVelocity;
                targetAccel = (manualVelocity - previousVelocity) / dt;
                manualDirection = 0; // precisa chamar rotate() de novo no proximo loop
                break;
            }
            case HOLD:
            case LOCKED:
                targetVelocity = 0;
                targetAccel = 0;
                break;
            case IDLE:
            default:
                break;
        }

        // 4) controle
        if (state == State.IDLE) {
            settled = false;
            error = 0;
            // parada ativa: se ainda esta girando, freia (so o BRAKE deixaria a torreta deslizar dezenas de graus)
            double brake = Math.abs(velocity) > IDLE_BRAKE_SPEED ? -IDLE_BRAKE * velocity : 0;
            setPower(Range.clip(brake, -MAX_POWER, MAX_POWER));
        } else {
            setPower(computePower());
        }

        telemetry.addData("Torre estado", state);
        telemetry.addData("Torre angulo / alvo", "%.1f / %.1f", angle, targetAngle);
        telemetry.addData("Torre erro", "%.1f%s", error, settled ? " (assentada)" : "");
        telemetry.addData("Torre potencia", "%.2f", power);
        if (targetAtLimit) {
            telemetry.addLine("Torre no LIMITE: colmeia fora do alcance de giro");
        }
    }

    private void updateRobotVelocity(Pose pose, double dt) {
        double x = pose.x(), y = pose.y(), heading = pose.heading();
        if (hasLastPose) {
            double a = VELOCITY_SMOOTHING;
            double vx = (x - lastX) / dt;
            double vy = (y - lastY) / dt;
            double omega = Math.toDegrees(normalizeRadians(heading - lastHeading)) / dt;
            robotVx += a * (vx - robotVx);
            robotVy += a * (vy - robotVy);
            double newOmega = robotOmega + a * (omega - robotOmega);
            robotAlpha += a * ((newOmega - robotOmega) / dt - robotAlpha);
            robotOmega = newOmega;
        }
        // guarda numeros (e nao o objeto Pose, que pode ser reaproveitado pela biblioteca)
        lastX = x;
        lastY = y;
        lastHeading = heading;
        hasLastPose = true;
    }

    private void computeTrackingTarget(Pose pose) {
        double heading = pose.heading();
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double turretX = pose.x() + TURRET_OFFSET_FORWARD * cos - TURRET_OFFSET_LEFT * sin;
        double turretY = pose.y() + TURRET_OFFSET_FORWARD * sin + TURRET_OFFSET_LEFT * cos;

        double dx = goal.x() - turretX;
        double dy = goal.y() - turretY;
        double distSq = Math.max(dx * dx + dy * dy, 1.0);

        // angulo que a torreta precisa ter em relacao a frente do robo
        double desired = Math.toDegrees(Math.atan2(dy, dx) - heading);

        // escolhe a volta (+-360) que cabe no limite e fica mais perto de onde a torreta esta
        double nearest = angle + normalizeDegrees(desired - angle);
        double best = Double.NaN;
        for (double candidate : new double[] {nearest, nearest - 360, nearest + 360}) {
            if (Math.abs(candidate) <= MAX_SPIN &&
                    (Double.isNaN(best) || Math.abs(candidate - angle) < Math.abs(best - angle))) {
                best = candidate;
            }
        }
        targetAtLimit = Double.isNaN(best);
        targetAngle = targetAtLimit ? Range.clip(nearest, -MAX_SPIN, MAX_SPIN) : best;

        if (targetAtLimit) {
            targetVelocity = 0;
            targetAccel = 0;
        } else {
            // velocidade que a torreta precisa ter para continuar mirando:
            // (quanto a direcao da colmeia muda porque o robo anda) - (quanto o robo gira)
            double bearingRate = Math.toDegrees((dy * robotVx - dx * robotVy) / distSq);
            targetVelocity = bearingRate - robotOmega;
            targetAccel = -robotAlpha;
        }
    }

    private double computePower() {
        error = targetAngle - angle;
        double vRef = FEEDFORWARD ? targetVelocity : 0;
        double aRef = FEEDFORWARD ? targetAccel : 0;
        boolean targetMoving = Math.abs(vRef) > TARGET_MOVING;

        // zona morta com histerese: assenta perto do alvo e so acorda se o erro crescer
        if (settled) {
            if (Math.abs(error) > RESUME || targetMoving) {
                settled = false;
            }
        } else if (Math.abs(error) < TOLERANCE && Math.abs(velocity) < SETTLE_SPEED && !targetMoving) {
            settled = true;
        }

        double out = 0;
        if (!settled) {
            out = kP * error + kD * (vRef - velocity) + kV * vRef + kA * aRef;
            if (out != 0) {
                out += Math.copySign(kS, out); // vence o atrito no sentido em que vamos empurrar
            }
            out *= NOMINAL_VOLTAGE / Math.max(voltage, 8.0); // bateria cheia ou fraca, mesmo resultado
        }
        out = Range.clip(out, -MAX_POWER, MAX_POWER);

        // seguranca: nunca empurrar ainda mais para fora do limite de giro
        if ((angle > MAX_SPIN + 5 && out > 0) || (angle < -MAX_SPIN - 5 && out < 0)) {
            out = 0;
        }
        return out;
    }

    private void setPower(double p) {
        power = p;
        // so manda para o hub se mudou (cada escrita custa alguns ms de loop)
        if (Double.isNaN(lastSentPower) || Math.abs(p - lastSentPower) > 0.002 || (p == 0 && lastSentPower != 0)) {
            motor.setPower(p);
            lastSentPower = p;
        }
    }

    // ===================== CONSULTAS =====================

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public boolean isTracking() {
        return state == State.TRACKING;
    }

    public boolean isLocked() {
        return state == State.LOCKED;
    }

    /** true quando esta mirando na colmeia com erro pequeno: use para liberar o disparo. */
    public boolean isOnTarget() {
        return state == State.TRACKING && !targetAtLimit && Math.abs(error) < AIM_TOLERANCE;
    }

    public boolean isSettled() {
        return settled;
    }

    /** Angulo da torreta em graus (0 = frente, positivo = esquerda). Atualizado no update(). */
    public double getAngle() {
        return angle;
    }

    /** Mantido para compatibilidade com o codigo antigo. */
    public double dislocation() {
        return getAngle();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public double getRobotOmega() {
        return robotOmega;
    }

    // ===================== UTIL =====================

    private static double normalizeDegrees(double degrees) {
        degrees %= 360;
        if (degrees > 180) degrees -= 360;
        if (degrees <= -180) degrees += 360;
        return degrees;
    }

    private static double normalizeRadians(double radians) {
        radians %= 2 * Math.PI;
        if (radians > Math.PI) radians -= 2 * Math.PI;
        if (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
