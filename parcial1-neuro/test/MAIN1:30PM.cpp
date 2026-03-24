// ================================================================
// NEUROROBOT DIFERENCIAL — main.cpp (ESP32 grande)
// Parcial 1 — Neurocontrol 2026
//   P1: Detección de terreno + giro autónomo >45°
//   P2: Aproximación neuromotora al objetivo (±1 cm)
//   P3: Árbitro neuronal — modula evasión vs atracción
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================================================================
// PINOUT — MOTORES (TB6612FNG × 2)
// ================================================================
const int M1_PWM = 14, M1_A = 26, M1_B = 17;
const int M2_PWM = 32, M2_A = 33, M2_B = 27;
const int STBY_IZQ = 25;

const int M3_PWM = 5,  M3_A = 4,  M3_B = 2;   // GPIO 2  — LOW en boot
const int M4_PWM = 13, M4_A = 12, M4_B = 16;  // GPIO 12 — NUNCA HIGH en boot
const int STBY_DER = 15;                        // GPIO 15 — NO HIGH en boot

const int CH_M1 = 0, CH_M2 = 1, CH_M3 = 2, CH_M4 = 3;

// ================================================================
// PINOUT — ULTRASONIDOS
// ================================================================
const int TRIG_FRONTAL  = 23, ECHO_FRONTAL  = 34;
const int TRIG_OBJETIVO = 0,  ECHO_OBJETIVO = 35;

// ================================================================
// PINOUT — IMU MPU-6050 (I2C bus 1)
// ================================================================
TwoWire I2C_IMU = TwoWire(1);
const int SDA_IMU = 18, SCL_IMU = 19;
const int MPU_ADDR = 0x68;

// ================================================================
// PINOUT — SERIAL2 → ESP32-S3 Mini (WiFi)
// ================================================================
#define TX_S3  22
#define RX_S3  36

// ================================================================
// KALMAN + ÁNGULOS
// ================================================================
float gyroOffset[3]  = {0, 0, 0};

float roll           = 0, rollUnc  = 4.0f;
float pitch          = 0, pitchUnc = 4.0f;
float yaw            = 0;

unsigned long tIMU   = 0;
bool imuOk           = false;

// ================================================================
// RED NEURONAL
// n[0] terreno  n[1] proxIzq  n[2] proxDer  n[3] parada
// n[4] integrador  n[5] oscA  n[6] oscB
// ================================================================
float n[7]     = {0};
float mem      = 0;   // memoria de terreno (τ lento)
float nAtr     = 0;   // atracción al objetivo
float nArb     = 0;   // árbitro evasión vs atracción

#define N_TERRENO   0
#define N_PROX_IZQ  1
#define N_PROX_DER  2
#define N_PARADA    3
#define N_INTEGRADOR 4
#define N_OSC_A     5
#define N_OSC_B     6

// ================================================================
// PARÁMETROS FÍSICOS
// ================================================================
const float DIST_PARADA   = 5.0f;
const float DIST_OBJETIVO = 5.0f;
const float TOLERANCIA    = 1.0f;

const int   PWM_CRUCERO   = 150;
const int   PWM_GIRO      = 200;

const float GRADOS_GIRO   = 45.0f;
const float THR_ROLL      = 5.0f;
const float THR_PITCH     = 5.0f;

// ================================================================
// CONSTANTES DE TIEMPO (τ)
// ================================================================
const float DT   = 10.0f;

const float TAU_TERRENO    = 25.0f;
const float TAU_PROX       = 18.0f;
const float TAU_PARADA     =  8.0f;
const float TAU_INTEGRADOR = 15.0f;
const float TAU_MEM        = 60.0f;
const float TAU_OSC_A      = 12.0f;
const float TAU_OSC_B      = 20.0f;
const float TAU_ATR        = 20.0f;
const float TAU_ARB        = 10.0f;

const float HILL_A = 100.0f;
const float HILL_B = 120.0f;
const float THR_PARADA = 60.0f;

// ================================================================
// PESOS SINÁPTICOS
// CAUTELOSO: W_EVA alto, W_ATR bajo
// TEMERARIO: W_EVA bajo, W_ATR alto
// Restricción: W_EVA + W_ATR = 1.0
// ================================================================

// Evasión
float W_TERRENO    = 0.8f;   // terreno → integrador
float W_PROX       = 0.4f;   // proximidad → integrador
float W_MEM_OSC    = 0.6f;   // memoria → osciladoras
float W_OSC_MUT    = 0.4f;   // excitación mutua osciladoras
float W_INH_OSC    = 0.2f;   // integrador inhibe osciladoras
float W_RETRO      = 0.15f;  // osciladoras → terreno (inercia)

// Atracción
float W_OBJ        = 0.9f;   // señal objetivo → nAtr
float W_ATR_MOT    = 0.85f;  // nAtr → velocidad motores

// Árbitro
float W_EVA        = 0.6f;   // peso evasión en árbitro
float W_ATR        = 0.4f;   // peso atracción en árbitro

// ================================================================
// PRESETS
// ================================================================
void modoCauteloso() {
    W_EVA = 0.85f; W_ATR = 0.15f;
    W_TERRENO = 1.1f; W_MEM_OSC = 0.75f; W_RETRO = 0.20f;
    Serial.println("[MODO] CAUTELOSO"); Serial2.println("MODO:CAUTELOSO");
}

void modoTemerario() {
    W_EVA = 0.25f; W_ATR = 0.75f;
    W_TERRENO = 0.4f; W_MEM_OSC = 0.3f; W_RETRO = 0.08f;
    Serial.println("[MODO] TEMERARIO"); Serial2.println("MODO:TEMERARIO");
}

void modoEquilibrado() {
    W_EVA = 0.6f; W_ATR = 0.4f;
    W_TERRENO = 0.8f; W_MEM_OSC = 0.6f; W_RETRO = 0.15f;
    Serial.println("[MODO] EQUILIBRADO"); Serial2.println("MODO:EQUILIBRADO");
}

// ================================================================
// MÁQUINA DE ESTADOS
// ================================================================
enum Estado { RECTO, GIRANDO, TERRENO_RUGOSO, APROX_OBJETIVO, META_OK };
Estado estado      = RECTO;
float  yawGiroInicio = 0;

// ================================================================
// HILL — función de activación
// ================================================================
inline float hill(float x) {
    return (HILL_A * x * x) / (HILL_B * HILL_B + x * x);
}

// ================================================================
// IMU
// ================================================================
bool initIMU() {
    I2C_IMU.beginTransmission(MPU_ADDR);
    I2C_IMU.write(0x6B); I2C_IMU.write(0x00);
    if (I2C_IMU.endTransmission() != 0) return false;
    delay(100);
    I2C_IMU.beginTransmission(MPU_ADDR);
    I2C_IMU.write(0x1A); I2C_IMU.write(0x05);
    I2C_IMU.endTransmission();
    return true;
}

bool readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {
    I2C_IMU.beginTransmission(MPU_ADDR);
    I2C_IMU.write(0x3B);
    if (I2C_IMU.endTransmission(false) != 0) return false;
    I2C_IMU.requestFrom(MPU_ADDR, 14);
    if (I2C_IMU.available() < 14) return false;
    auto rd = [&]() -> int16_t {
        return (I2C_IMU.read() << 8) | I2C_IMU.read();
    };
    ax = rd()/4096.0f; ay = rd()/4096.0f; az = rd()/4096.0f;
    rd();
    gx = rd()/65.5f; gy = rd()/65.5f; gz = rd()/65.5f;
    return true;
}

void kalman(float &ang, float &unc, float rate, float meas, float dt) {
    ang += dt * rate;
    unc += dt * dt * 16.0f;
    float g = unc / (unc + 9.0f);
    ang += g * (meas - ang);
    unc  = (1.0f - g) * unc;
}

// ================================================================
// MOTORES
// ================================================================
void setMotor(int ch, int pinA, int pinB, int v) {
    v = constrain(v, -255, 255);
    digitalWrite(pinA, v > 0 ? HIGH : LOW);
    digitalWrite(pinB, v < 0 ? HIGH : LOW);
    ledcWrite(ch, abs(v));
}

void setMotores(int vI, int vD) {
    setMotor(CH_M1, M1_A, M1_B, vI);
    setMotor(CH_M3, M3_A, M3_B, vI);
    setMotor(CH_M2, M2_A, M2_B, vD);
    setMotor(CH_M4, M4_A, M4_B, vD);
}

void frenar()       { setMotores(0, 0); }
void avanzar()      { setMotores(PWM_CRUCERO, PWM_CRUCERO); }
void girar()        { setMotores(PWM_GIRO, -PWM_GIRO); }
void acercarse(float d) {
    int pwm = (int)map(constrain((long)d, 6, 200), 6, 200, 40, PWM_CRUCERO);
    setMotores(pwm, pwm);
}

// ================================================================
// ULTRASONIDO
// ================================================================
float dist(int trig, int echo) {
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long t = pulseIn(echo, HIGH, 15000);
    return t == 0 ? 999.0f : t * 0.01723f;
}

// ================================================================
// NEUROCONTROLADOR
// ================================================================
void neuroStep(float imuSig, float dFront, float dLat, float dObj) {

    float retro = (n[N_OSC_A] + n[N_OSC_B]) * W_RETRO;

    n[N_TERRENO] += (DT/TAU_TERRENO) * (
        -n[N_TERRENO] + hill(imuSig) + retro );

    float sF = 100.0f / max(dFront, 0.1f);
    float sL = 100.0f / max(dLat,   0.1f);
    n[N_PROX_IZQ] += (DT/TAU_PROX) * ( -n[N_PROX_IZQ] + hill(sF) );
    n[N_PROX_DER] += (DT/TAU_PROX) * ( -n[N_PROX_DER] + hill(sL) );

    bool obstaculo = (dFront <= DIST_PARADA || dLat <= DIST_PARADA);
    n[N_PARADA] += (DT/TAU_PARADA) * ( -n[N_PARADA] + (obstaculo ? 100.0f : 0.0f) );

    float nTerrNorm = n[N_TERRENO] / (n[N_TERRENO] + 1.0f);
    mem += (DT/TAU_MEM) * ( -mem + nTerrNorm * 100.0f );

    n[N_INTEGRADOR] += (DT/TAU_INTEGRADOR) * (
        -n[N_INTEGRADOR]
        + n[N_TERRENO]   * W_TERRENO
        + n[N_PROX_IZQ]  * W_PROX
        + n[N_PROX_DER]  * W_PROX );

    n[N_OSC_A] += (DT/TAU_OSC_A) * (
        -n[N_OSC_A]
        + mem        * W_MEM_OSC
        + n[N_OSC_B] * W_OSC_MUT
        - n[N_INTEGRADOR] * W_INH_OSC );

    n[N_OSC_B] += (DT/TAU_OSC_B) * (
        -n[N_OSC_B]
        + mem        * W_MEM_OSC
        + n[N_OSC_A] * W_OSC_MUT
        - n[N_INTEGRADOR] * W_INH_OSC );

    bool objEnRango = (dObj > DIST_OBJETIVO && dObj < 200.0f);
    float sObj = objEnRango
        ? hill(100.0f / max(dObj, 0.1f)) * W_OBJ
        : 0.0f;
    nAtr += (DT/TAU_ATR) * ( -nAtr + sObj );

    nArb += (DT/TAU_ARB) * (
        -nArb
        + n[N_INTEGRADOR] * W_EVA
        + nAtr             * W_ATR );

    for (int i = 0; i < 7; i++) n[i] = constrain(n[i], 0, 100);
    mem  = constrain(mem,  0, 100);
    nAtr = constrain(nAtr, 0, 100);
    nArb = constrain(nArb, 0, 100);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== NEUROROBOT INICIANDO ===");

    int salidas[] = {
        M1_A, M1_B, M1_PWM,
        M2_A, M2_B, M2_PWM,
        M3_A,        M3_PWM,
               M4_B, M4_PWM,
        STBY_IZQ, STBY_DER,
        TRIG_FRONTAL, TRIG_OBJETIVO
    };
    for (int p : salidas) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

    pinMode(M3_B, OUTPUT); digitalWrite(M3_B, LOW);  // GPIO 2
    pinMode(M4_A, OUTPUT); digitalWrite(M4_A, LOW);  // GPIO 12

    pinMode(ECHO_FRONTAL,  INPUT);
    pinMode(ECHO_OBJETIVO, INPUT);

    ledcSetup(CH_M1, 5000, 8); ledcAttachPin(M1_PWM, CH_M1);
    ledcSetup(CH_M2, 5000, 8); ledcAttachPin(M2_PWM, CH_M2);
    ledcSetup(CH_M3, 5000, 8); ledcAttachPin(M3_PWM, CH_M3);
    ledcSetup(CH_M4, 5000, 8); ledcAttachPin(M4_PWM, CH_M4);

    delay(10);
    digitalWrite(STBY_IZQ, HIGH);
    digitalWrite(STBY_DER, HIGH);
    Serial.println("[OK] Motores");

    I2C_IMU.begin(SDA_IMU, SCL_IMU, 400000);
    imuOk = initIMU();
    Serial.println(imuOk ? "[OK] IMU" : "[ERROR] IMU");

    tIMU = micros();
    modoEquilibrado(); //MOVER MODO

    Serial.println("\nEstado          | dFront  dObj | Roll  Pitch   Yaw");
    Serial.println("N0_terr  N1_pIzq  N2_pDer  N3_stop  N4_integ  N5_oscA  N6_oscB  Mem  Atr  Arb  Giro");
    Serial.println("----------------|-------------|-------------------|------------------------------------------------------------------------");

    Serial.println("Esperando Serial2 (2s)...");
    delay(2000);
    Serial2.begin(115200, SERIAL_8N1, RX_S3, TX_S3);
    Serial.println("[OK] Serial2 → S3 Mini (TX=G22 RX=G36)");
}

// ================================================================
// LOOP (~66 Hz)
// ================================================================
void loop() {
    
    static unsigned long tLoop = 0;
    if (millis() - tLoop < 15) return;
    tLoop = millis();
// Al inicio del loop — leer comando por Serial
if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'C') modoCauteloso();
    if (cmd == 'T') modoTemerario();
    if (cmd == 'E') modoEquilibrado();
}
    static unsigned long tHB = 0;
    if (millis() - tHB > 5000) {
        tHB = millis();
        Serial2.println("HB:VIVO");
    }

    // 1. Sensores
    float dFront = dist(TRIG_FRONTAL,  ECHO_FRONTAL);
    float dObj   = dist(TRIG_OBJETIVO, ECHO_OBJETIVO);

    // 2. IMU + Kalman
    float imuSig = 0;
    if (imuOk) {
        float ax, ay, az, gx, gy, gz;
        if (readIMU(ax, ay, az, gx, gy, gz)) {
            float dt = (micros() - tIMU) / 1e6f;
            tIMU = micros();

            float rawRoll  =  atan2f(ay, sqrtf(ax*ax + az*az)) * 57.3f;
            float rawPitch = -atan2f(ax, sqrtf(ay*ay + az*az)) * 57.3f;

            kalman(roll,  rollUnc,  gx - gyroOffset[0], rawRoll,  dt);
            kalman(pitch, pitchUnc, gy - gyroOffset[1], rawPitch, dt);
            yaw += (gz - gyroOffset[2]) * dt;

            bool terrenoActivo = (fabsf(roll) > THR_ROLL || fabsf(pitch) > THR_PITCH);
            imuSig = terrenoActivo
                ? min(55.0f, fabsf(roll) * 2.5f + fabsf(pitch) * 1.5f)
                : 0.0f;
        }
    }

    // 3. Red neuronal
    neuroStep(imuSig, dFront, dFront, dObj);

    // 4. Decisión
    bool enParada  = (n[N_PARADA] > THR_PARADA);
    bool terrenoOn = (fabsf(roll) > THR_ROLL || fabsf(pitch) > THR_PITCH);
    bool metaOk    = (dObj >= DIST_OBJETIVO - TOLERANCIA &&
                      dObj <= DIST_OBJETIVO + TOLERANCIA);

    if (enParada) {
        frenar();
        estado = RECTO;

    } else if (metaOk) {
        frenar();
        estado = META_OK;
        Serial.println(">> META ALCANZADA");

    } else {
        float fEva = (n[N_INTEGRADOR] * W_EVA) / max(nArb, 0.001f);
        float fAtr = (nAtr            * W_ATR) / max(nArb, 0.001f);

        if (fEva >= fAtr) {
            // Evasión — P1
            switch (estado) {
                case RECTO:
                    if (terrenoOn) {
                        yawGiroInicio = yaw;
                        estado = GIRANDO;
                        Serial.printf(">> GIRO yaw=%.1f\n", yaw);
                        girar();
                    } else avanzar();
                    break;

                case GIRANDO:
                    if (fabsf(yaw - yawGiroInicio) >= GRADOS_GIRO) {
                        estado = TERRENO_RUGOSO;
                        Serial.println(">> 45° OK");
                        avanzar();
                    } else girar();
                    break;

                case TERRENO_RUGOSO:
                    if (!terrenoOn) {
                        estado = RECTO;
                        Serial.println(">> Terreno liso");
                    }
                    avanzar();
                    break;

                default:
                    estado = RECTO;
                    avanzar();
                    break;
            }
        } else {
            // Atracción — P2
            estado = APROX_OBJETIVO;
            acercarse(dObj);
        }
    }

    // 5. Telemetría
    static unsigned long tPrint = 0;
    if (millis() - tPrint < 150) return;
    tPrint = millis();

    const char* stNombre =
        enParada            ? "PARADA      " :
        estado == META_OK   ? "META-OK     " :
        estado == GIRANDO   ? "GIRANDO     " :
        estado == TERRENO_RUGOSO  ? "TERRENO-RUG " :
        estado == APROX_OBJETIVO  ? "APROX-OBJ   " :
                                    "RECTO       ";

    if (!imuOk) {
        Serial.println("[ERROR] IMU desconectada");
    } else {
        Serial.printf("%s | %5.1f %5.1f | R:%5.1f P:%5.1f Y:%6.1f\n",
            stNombre, dFront, dObj, roll, pitch, yaw);

        Serial.printf(
            "  N0:%4.1f  N1:%4.1f  N2:%4.1f  N3:%4.1f"
            "  N4:%4.1f  N5:%4.1f  N6:%4.1f"
            "  Mem:%4.1f  Atr:%4.1f  Arb:%4.1f  Giro:%.1f\n",
            n[N_TERRENO], n[N_PROX_IZQ], n[N_PROX_DER], n[N_PARADA],
            n[N_INTEGRADOR], n[N_OSC_A], n[N_OSC_B],
            mem, nAtr, nArb,
            fabsf(yaw - yawGiroInicio)
        );
        Serial.println("  ---");
    }

    Serial2.printf(
        "ST:%s,DF:%.1f,OBJ:%.1f,R:%.1f,P:%.1f,Y:%.1f,"
        "N0:%.1f,N1:%.1f,N2:%.1f,N3:%.1f,N4:%.1f,N5:%.1f,N6:%.1f,"
        "MEM:%.1f,ATR:%.1f,ARB:%.1f,GIRO:%.1f\n",
        stNombre, dFront, dObj, roll, pitch, yaw,
        n[N_TERRENO], n[N_PROX_IZQ], n[N_PROX_DER], n[N_PARADA],
        n[N_INTEGRADOR], n[N_OSC_A], n[N_OSC_B],
        mem, nAtr, nArb,
        fabsf(yaw - yawGiroInicio)
    );
}