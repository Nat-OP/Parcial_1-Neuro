// ================================================================
// NEUROROBOT DIFERENCIAL — main.cpp (ESP32 grande)
// Fixes red neuronal:
//   - neuroStep recibe distFront y distObj por separado (no duplicado)
//   - W_OSC_MUT reducido para evitar saturación de osciladoras
//   - W_INH_OSC aumentado para que el integrador las frene mejor
//   - Señal proximidad escalada para que N1/N2 respondan antes
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================================================================
// PINOUT — MOTORES
// ================================================================
const int M1_PWM = 33, M1_A = 26, M1_B = 25;
const int M2_PWM = 13, M2_A = 14, M2_B = 12;
const int STBY_IZQ = 27;

const int M3_PWM = 23, M3_A = 17, M3_B = 5;
const int M4_PWM = 32, M4_A =  4, M4_B = 0;
const int STBY_DER = 16;

const int CH_M1 = 0, CH_M2 = 1, CH_M3 = 2, CH_M4 = 3;

// ================================================================
// PINOUT — ULTRASONIDOS
// ================================================================
const int TRIG_FRONTAL  = 15, ECHO_FRONTAL  = 34;
const int TRIG_OBJETIVO =  2, ECHO_OBJETIVO = 35;

#define US_TIMEOUT_US 5000UL

// ================================================================
// PINOUT — IMU
// ================================================================
TwoWire I2C_IMU = TwoWire(1);
const int SDA_IMU = 18, SCL_IMU = 19;
const int MPU_ADDR = 0x68;

// ================================================================
// PINOUT — SERIAL2
// ================================================================
#define TX_S3 22
#define RX_S3 21

// ================================================================
// TELEMETRÍA — buffer compartido entre cores
// ================================================================
static char   telBuf[220] = {0};
static volatile bool telListo = false;
SemaphoreHandle_t telMutex;

void tareaSerial2(void*) {
    for (;;) {
        if (telListo) {
            if (xSemaphoreTake(telMutex, 0) == pdTRUE) {
                Serial2.print(telBuf);
                telListo = false;
                xSemaphoreGive(telMutex);
            }
        }
        vTaskDelay(1);
    }
}

// ================================================================
// KALMAN + ÁNGULOS
// ================================================================
float gyroOffset[3] = {0, 0, 0};
float roll = 0, rollUnc = 4.0f;
float pitch = 0, pitchUnc = 4.0f;
float yaw  = 0;
unsigned long tIMU = 0;
bool imuOk = false;

// ================================================================
// RED NEURONAL
// ================================================================
float n[7] = {0};
float mem  = 0;
float nAtr = 0;
float nArb = 0;

#define N_TERRENO    0
#define N_PROX_IZQ   1
#define N_PROX_DER   2
#define N_PARADA     3
#define N_INTEGRADOR 4
#define N_OSC_A      5
#define N_OSC_B      6

// ================================================================
// PARÁMETROS FÍSICOS
// ================================================================
const float DIST_PARADA   =  5.0f;
const float DIST_OBJETIVO =  5.0f;
const float TOLERANCIA    =  1.0f;
// ================================================================
// PWM INDIVIDUAL — ajusta cada motor por separado
// Cambia el signo para invertir dirección: ej. -150 gira al revés
// ================================================================
const int PWM_M1 = +60;    // Izquierda adelante  — cambia signo para invertir
const int PWM_M2 = +60;    // Derecha adelante    — cambia signo para invertir
const int PWM_M3 = +60;    // Izquierda adelante (Driver 2)
const int PWM_M4 = +60;    // Derecha adelante   (Driver 2)

const int PWM_GIRO_IZQ = +100;   // M1, M3 al girar
const int PWM_GIRO_DER = -100;   // M2, M4 al girar (opuesto)
const float GRADOS_GIRO   = 45.0f;
const float THR_ROLL      =  5.0f;
const float THR_PITCH     =  5.0f;
const float DT            = 10.0f;
const float TAU_TERRENO   = 25.0f;
const float TAU_PROX      = 18.0f;
const float TAU_PARADA_T  =  8.0f;
const float TAU_INTEGRADOR= 15.0f;
const float TAU_MEM       = 60.0f;
const float TAU_OSC_A     = 12.0f;
const float TAU_OSC_B     = 20.0f;
const float TAU_ATR       = 20.0f;
const float TAU_ARB       = 10.0f;
const float HILL_A        = 100.0f;
const float HILL_B        = 120.0f;
const float THR_PARADA_N  = 60.0f;

// ================================================================
// PESOS SINÁPTICOS
// Fix: W_OSC_MUT bajo para evitar saturación, W_INH_OSC mayor
// ================================================================
float W_TERRENO  = 0.8f;
float W_PROX     = 0.4f;
float W_MEM_OSC  = 0.6f;
float W_OSC_MUT  = 0.15f;   // era 0.4 — reducido para evitar saturación
float W_INH_OSC  = 0.35f;   // era 0.2 — aumentado para frenar osciladoras
float W_RETRO    = 0.15f;
float W_OBJ      = 0.9f;
float W_ATR_MOT  = 0.85f;
float W_EVA      = 0.6f;
float W_ATR      = 0.4f;

// ================================================================
// PRESETS
// ================================================================
void modoCauteloso() {
    W_EVA=0.85f; W_ATR=0.15f; W_TERRENO=1.1f; W_MEM_OSC=0.75f; W_RETRO=0.20f;
    Serial.println("[MODO] CAUTELOSO");
    if (xSemaphoreTake(telMutex, 5) == pdTRUE) {
        strcpy(telBuf, "MODO:CAUTELOSO\n"); telListo = true;
        xSemaphoreGive(telMutex);
    }
}
void modoTemerario() {
    W_EVA=0.25f; W_ATR=0.75f; W_TERRENO=0.4f; W_MEM_OSC=0.3f; W_RETRO=0.08f;
    Serial.println("[MODO] TEMERARIO");
    if (xSemaphoreTake(telMutex, 5) == pdTRUE) {
        strcpy(telBuf, "MODO:TEMERARIO\n"); telListo = true;
        xSemaphoreGive(telMutex);
    }
}
void modoEquilibrado() {
    W_EVA=0.6f; W_ATR=0.4f; W_TERRENO=0.8f; W_MEM_OSC=0.6f; W_RETRO=0.15f;
    Serial.println("[MODO] EQUILIBRADO");
    if (xSemaphoreTake(telMutex, 5) == pdTRUE) {
        strcpy(telBuf, "MODO:EQUILIBRADO\n"); telListo = true;
        xSemaphoreGive(telMutex);
    }
}

// ================================================================
// ESTADOS
// ================================================================
enum Estado { INICIALIZANDO, RECTO, GIRANDO, TERRENO_RUGOSO, APROX_OBJETIVO, META_OK };
Estado estado = INICIALIZANDO;
float  yawGiroInicio = 0;
unsigned long tInicializando = 0;
const unsigned long T_INIT_MS = 3000;

// ================================================================
// HILL
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
    ax=rd()/4096.0f; ay=rd()/4096.0f; az=rd()/4096.0f;
    rd();
    gx=rd()/65.5f; gy=rd()/65.5f; gz=rd()/65.5f;
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
// Cada motor usa su propio PWM declarado arriba
// El signo del PWM_Mx ya incluye la dirección correcta
void setMotores(bool adelante) {
    if (!adelante) {
        setMotor(CH_M1, M1_A, M1_B, 0);
        setMotor(CH_M2, M2_A, M2_B, 0);
        setMotor(CH_M3, M3_A, M3_B, 0);
        setMotor(CH_M4, M4_A, M4_B, 0);
        return;
    }
    setMotor(CH_M1, M1_A, M1_B,  PWM_M1);
    setMotor(CH_M2, M2_A, M2_B,  PWM_M2);
    setMotor(CH_M3, M3_A, M3_B,  PWM_M3);
    setMotor(CH_M4, M4_A, M4_B,  PWM_M4);
}
void frenar()  {
    setMotor(CH_M1, M1_A, M1_B, 0);
    setMotor(CH_M2, M2_A, M2_B, 0);
    setMotor(CH_M3, M3_A, M3_B, 0);
    setMotor(CH_M4, M4_A, M4_B, 0);
}
void avanzar() { setMotores(true); }
void girar()   {
    setMotor(CH_M1, M1_A, M1_B,  PWM_GIRO_IZQ);
    setMotor(CH_M2, M2_A, M2_B,  PWM_GIRO_DER);
    setMotor(CH_M3, M3_A, M3_B,  PWM_GIRO_IZQ);
    setMotor(CH_M4, M4_A, M4_B,  PWM_GIRO_DER);
}
void acercarse(float d) {
    // Escala proporcional al PWM de cada motor manteniendo la relación
    float ratio = (float)map(constrain((long)d, 6, 200), 6, 200, 40, 150) / 150.0f;
    setMotor(CH_M1, M1_A, M1_B, (int)(PWM_M1 * ratio));
    setMotor(CH_M2, M2_A, M2_B, (int)(PWM_M2 * ratio));
    setMotor(CH_M3, M3_A, M3_B, (int)(PWM_M3 * ratio));
    setMotor(CH_M4, M4_A, M4_B, (int)(PWM_M4 * ratio));
}

// ================================================================
// ULTRASONIDO
// ================================================================
float distFront = 999.0f, distObj = 999.0f;

void leerUltrasonidos() {
    digitalWrite(TRIG_FRONTAL, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_FRONTAL, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_FRONTAL, LOW);
    long t1 = pulseIn(ECHO_FRONTAL, HIGH, US_TIMEOUT_US);
    if (t1 > 0) distFront = t1 * 0.01723f;

    digitalWrite(TRIG_OBJETIVO, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_OBJETIVO, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_OBJETIVO, LOW);
    long t2 = pulseIn(ECHO_OBJETIVO, HIGH, US_TIMEOUT_US);
    if (t2 > 0) distObj = t2 * 0.01723f;
}

// ================================================================
// NEUROCONTROLADOR
// Fix principal: N1 recibe distFront, N2 recibe distObj (diferentes)
// Señal escalada: 30cm → señal útil, 5cm → saturación
// ================================================================
void neuroStep(float imuSig, float dFront, float dObj) {

    float retro = (n[N_OSC_A] + n[N_OSC_B]) * W_RETRO;
    n[N_TERRENO] += (DT/TAU_TERRENO) * (-n[N_TERRENO] + hill(imuSig) + retro);

    // Señal escalada: referencia 30cm para que hill() responda antes
    float sF = 30.0f / max(dFront, 0.1f) * 100.0f;   // 30cm→100, 5cm→600
    float sO = 30.0f / max(dObj,   0.1f) * 100.0f;
    n[N_PROX_IZQ] += (DT/TAU_PROX) * (-n[N_PROX_IZQ] + hill(sF));
    n[N_PROX_DER] += (DT/TAU_PROX) * (-n[N_PROX_DER] + hill(sO));

    bool obstaculo = (dFront <= DIST_PARADA || dObj <= DIST_PARADA);
    n[N_PARADA] += (DT/TAU_PARADA_T) * (-n[N_PARADA] + (obstaculo ? 100.0f : 0.0f));

    float nTerrNorm = n[N_TERRENO] / (n[N_TERRENO] + 1.0f);
    mem += (DT/TAU_MEM) * (-mem + nTerrNorm * 100.0f);

    n[N_INTEGRADOR] += (DT/TAU_INTEGRADOR) * (
        -n[N_INTEGRADOR]
        + n[N_TERRENO]  * W_TERRENO
        + n[N_PROX_IZQ] * W_PROX
        + n[N_PROX_DER] * W_PROX);

    // Osciladoras — W_OSC_MUT reducido para evitar saturación cruzada
    n[N_OSC_A] += (DT/TAU_OSC_A) * (
        -n[N_OSC_A]
        + mem        * W_MEM_OSC
        + n[N_OSC_B] * W_OSC_MUT
        - n[N_INTEGRADOR] * W_INH_OSC);

    n[N_OSC_B] += (DT/TAU_OSC_B) * (
        -n[N_OSC_B]
        + mem        * W_MEM_OSC
        + n[N_OSC_A] * W_OSC_MUT
        - n[N_INTEGRADOR] * W_INH_OSC);

    bool objEnRango = (dObj > DIST_OBJETIVO && dObj < 200.0f);
    float sObj = objEnRango ? hill(100.0f / max(dObj, 0.1f)) * W_OBJ : 0.0f;
    nAtr += (DT/TAU_ATR) * (-nAtr + sObj);

    nArb += (DT/TAU_ARB) * (-nArb + n[N_INTEGRADOR] * W_EVA + nAtr * W_ATR);

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

    // Strapping pins — LOW antes que todo
    pinMode(M2_B, OUTPUT); digitalWrite(M2_B, LOW);   // GPIO 12
    pinMode(M4_B, OUTPUT); digitalWrite(M4_B, LOW);   // GPIO 0

    // LEDC primero — attach antes de cualquier pinMode en pins PWM
    ledcSetup(CH_M1, 5000, 8); ledcAttachPin(M1_PWM, CH_M1);
    ledcSetup(CH_M2, 5000, 8); ledcAttachPin(M2_PWM, CH_M2);
    ledcSetup(CH_M3, 5000, 8); ledcAttachPin(M3_PWM, CH_M3);
    ledcSetup(CH_M4, 5000, 8); ledcAttachPin(M4_PWM, CH_M4);

    // Arrancar todos los canales en 0
    ledcWrite(CH_M1, 0); ledcWrite(CH_M2, 0);
    ledcWrite(CH_M3, 0); ledcWrite(CH_M4, 0);

    // Pines de direccion — PWM pins NO incluidos
    int salidas[] = {
        M1_A, M1_B,
        M2_A,
        M3_A, M3_B,
        M4_A,                  // M4_B (GPIO0) ya fue puesto LOW arriba
        STBY_IZQ, STBY_DER,
        TRIG_FRONTAL, TRIG_OBJETIVO
    };
    for (int p : salidas) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

    pinMode(ECHO_FRONTAL,  INPUT);
    pinMode(ECHO_OBJETIVO, INPUT);

    digitalWrite(STBY_IZQ, LOW);
    digitalWrite(STBY_DER, LOW);
    Serial.println("[OK] Motores (deshabilitados hasta init)");

    I2C_IMU.begin(SDA_IMU, SCL_IMU, 400000);
    imuOk = initIMU();
    Serial.println(imuOk ? "[OK] IMU" : "[ERROR] IMU");
    tIMU = micros();

    telMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(tareaSerial2, "Serial2Task", 2048, NULL, 1, NULL, 0);

    delay(2000);
    Serial2.begin(115200, SERIAL_8N1, RX_S3, TX_S3);
    Serial.println("[OK] Serial2 → S3 Mini (TX=G22 RX=G21)");

    tInicializando = millis();
    Serial.printf("[INIT] Esperando %lu ms...\n", T_INIT_MS);
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    static unsigned long tLoop = 0;
    if (millis() - tLoop < 15) return;
    tLoop = millis();

    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'C') modoCauteloso();
        if (cmd == 'T') modoTemerario();
        if (cmd == 'E') modoEquilibrado();
    }

    // ── INICIALIZANDO ────────────────────────────────────────
    if (estado == INICIALIZANDO) {
        frenar();
        unsigned long elapsed = millis() - tInicializando;
        static unsigned long tCuenta = 0;
        if (millis() - tCuenta > 500) {
            tCuenta = millis();
            Serial.printf("[INIT] %lu / %lu ms\n", elapsed, T_INIT_MS);
        }
        if (elapsed >= T_INIT_MS) {
            digitalWrite(STBY_IZQ, HIGH);
            digitalWrite(STBY_DER, HIGH);
            modoEquilibrado();
            estado = RECTO;
            Serial.println(">> INIT OK — motores ON");
        }
        return;
    }

    // ── SENSORES ─────────────────────────────────────────────
    leerUltrasonidos();

    // ── IMU ──────────────────────────────────────────────────
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
                ? min(55.0f, fabsf(roll)*2.5f + fabsf(pitch)*1.5f)
                : 0.0f;
        }
    }

    // ── RED NEURONAL — ahora con distFront y distObj separados ──
    neuroStep(imuSig, distFront, distObj);

    // ── DECISIÓN ─────────────────────────────────────────────
    bool enParada  = (n[N_PARADA] > THR_PARADA_N);
    bool terrenoOn = (fabsf(roll) > THR_ROLL || fabsf(pitch) > THR_PITCH);
    bool metaOk    = (distObj >= DIST_OBJETIVO - TOLERANCIA &&
                      distObj <= DIST_OBJETIVO + TOLERANCIA);

    if (enParada) {
        frenar(); estado = RECTO;
    } else if (metaOk) {
        frenar(); estado = META_OK;
        Serial.println(">> META ALCANZADA");
    } else {
        float fEva = (n[N_INTEGRADOR] * W_EVA) / max(nArb, 0.001f);
        float fAtr = (nAtr            * W_ATR) / max(nArb, 0.001f);

        if (fEva >= fAtr) {
            switch (estado) {
                case RECTO:
                    if (terrenoOn) {
                        yawGiroInicio = yaw; estado = GIRANDO;
                        Serial.printf(">> GIRO yaw=%.1f\n", yaw);
                        girar();
                    } else avanzar();
                    break;
                case GIRANDO:
                    if (fabsf(yaw - yawGiroInicio) >= GRADOS_GIRO) {
                        estado = TERRENO_RUGOSO;
                        Serial.println(">> 45 OK");
                        avanzar();
                    } else girar();
                    break;
                case TERRENO_RUGOSO:
                    if (!terrenoOn) { estado = RECTO; Serial.println(">> Terreno liso"); }
                    avanzar();
                    break;
                default: estado = RECTO; avanzar(); break;
            }
        } else {
            estado = APROX_OBJETIVO;
            acercarse(distObj);
        }
    }

    // ── TELEMETRÍA USB ────────────────────────────────────────
    static unsigned long tPrint = 0;
    if (millis() - tPrint < 150) return;
    tPrint = millis();

    const char* stNombre =
        enParada                 ? "PARADA      " :
        estado == META_OK        ? "META-OK     " :
        estado == GIRANDO        ? "GIRANDO     " :
        estado == TERRENO_RUGOSO ? "TERRENO-RUG " :
        estado == APROX_OBJETIVO ? "APROX-OBJ   " :
                                   "RECTO       ";

    if (!imuOk) {
        Serial.println("[ERROR] IMU desconectada");
    } else {
        Serial.printf("%s | %5.1f %5.1f | R:%5.1f P:%5.1f Y:%6.1f\n",
            stNombre, distFront, distObj, roll, pitch, yaw);
        Serial.printf(
            "  N0:%4.1f N1:%4.1f N2:%4.1f N3:%4.1f"
            " N4:%4.1f N5:%4.1f N6:%4.1f"
            " Mem:%4.1f Atr:%4.1f Arb:%4.1f Giro:%.1f\n",
            n[N_TERRENO], n[N_PROX_IZQ], n[N_PROX_DER], n[N_PARADA],
            n[N_INTEGRADOR], n[N_OSC_A], n[N_OSC_B],
            mem, nAtr, nArb, fabsf(yaw - yawGiroInicio));
        Serial.println("  ---");
    }

    // ── TELEMETRÍA S3 — no bloqueante ─────────────────────────
    if (!telListo) {
        if (xSemaphoreTake(telMutex, 0) == pdTRUE) {
            snprintf(telBuf, sizeof(telBuf),
                "ST:%s,DF:%.1f,OBJ:%.1f,R:%.1f,P:%.1f,Y:%.1f,"
                "N0:%.1f,N1:%.1f,N2:%.1f,N3:%.1f,N4:%.1f,N5:%.1f,N6:%.1f,"
                "MEM:%.1f,ATR:%.1f,ARB:%.1f,GIRO:%.1f\n",
                stNombre, distFront, distObj, roll, pitch, yaw,
                n[N_TERRENO], n[N_PROX_IZQ], n[N_PROX_DER], n[N_PARADA],
                n[N_INTEGRADOR], n[N_OSC_A], n[N_OSC_B],
                mem, nAtr, nArb, fabsf(yaw - yawGiroInicio));
            telListo = true;
            xSemaphoreGive(telMutex);
        }
    }
}
