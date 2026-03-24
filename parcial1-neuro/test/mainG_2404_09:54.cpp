#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================================================================
// PINOUT — MOTORES Y DRIVERS
// ================================================================
const int M1_PWM = 33, M1_A = 26, M1_B = 25;
const int M2_PWM = 13, M2_A = 14, M2_B = 12;
const int STBY_IZQ = 27;

const int M3_PWM =  0, M3_A = 16, M3_B =  4;
const int M4_PWM = 32, M4_A =  5, M4_B = 23;
const int STBY_DER = 17;

const int CH_M1 = 0, CH_M2 = 1, CH_M3 = 2, CH_M4 = 3;

// ================================================================
// PINOUT — SENSORES Y COMUNICACIÓN
// ================================================================
const int TRIG_FRONTAL  = 15, ECHO_FRONTAL  = 34;
const int TRIG_OBJETIVO =  2, ECHO_OBJETIVO = 35;
#define US_TIMEOUT_US 5000UL

// I2C IMU
TwoWire I2C_IMU = TwoWire(1);
const int SDA_IMU = 18, SCL_IMU = 19;
const int MPU_ADDR = 0x68;

// SERIAL2 — TELEMETRÍA S3 MINI
#define TX_S3 22
#define RX_S3 21

// ================================================================
// VARIABLES GLOBALES (CORREGIDAS)
// ================================================================
// --- Sensores ---
float distFront = 999.0f; // <--- Faltaban estas dos
float distObj = 999.0f;   // <--- declaraciones globales

// --- IMU y Kalman ---
float roll = 0, rollUnc = 4.0f, pitch = 0, pitchUnc = 4.0f, yaw = 0;
float gyroOffset[3] = {0, 0, 0};
unsigned long tIMU = 0;
bool imuOk = false;

// --- Red Neuronal ---
float n[7] = {0}, mem = 0, nAtr = 0, nArb = 0;
#define N_TERRENO 0
#define N_PROX_IZQ 1
#define N_PROX_DER 2
#define N_PARADA 3
#define N_INTEGRADOR 4
#define N_OSC_A 5
#define N_OSC_B 6

// ================================================================
// TELEMETRÍA — BUFFER Y TAREA
// ================================================================
static char    telBuf[250] = {0};
static volatile bool telListo = false;
SemaphoreHandle_t telMutex;

void tareaSerial2(void* pvParameters) {
    for (;;) {
        if (telListo) {
            if (xSemaphoreTake(telMutex, portMAX_DELAY) == pdTRUE) {
                Serial2.print(telBuf);
                telListo = false;
                xSemaphoreGive(telMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ================================================================
// PARÁMETROS Y ESTADOS
// ================================================================
enum Estado { INICIALIZANDO, RECTO, GIRANDO, TERRENO_RUGOSO, APROX_OBJETIVO, META_OK };
Estado estado = INICIALIZANDO;
float yawGiroInicio = 0;

const float DIST_PARADA = 5.0f, DIST_OBJETIVO = 5.0f, TOLERANCIA = 1.0f, GRADOS_GIRO = 45.0f;
const float THR_ROLL = 5.0f, THR_PITCH = 5.0f, DT = 10.0f, HILL_A = 100.0f, HILL_B = 120.0f, THR_PARADA_N = 60.0f;
const int PWM_M1=60, PWM_M2=60, PWM_M3=60, PWM_M4=60;
const int PWM_GIRO_M1=100, PWM_GIRO_M2=-100, PWM_GIRO_M3=-100, PWM_GIRO_M4=100;

const float TAU_TERRENO=25, TAU_PROX=18, TAU_PARADA_T=8, TAU_INTEGRADOR=15, TAU_MEM=60, TAU_OSC_A=12, TAU_OSC_B=20, TAU_ATR=20, TAU_ARB=10;
float W_TERRENO=0.8f, W_PROX=0.4f, W_MEM_OSC=0.6f, W_OSC_MUT=0.15f, W_INH_OSC=0.35f, W_RETRO=0.15f, W_OBJ=0.9f, W_EVA=0.6f, W_ATR=0.4f;

// ================================================================
// FUNCIONES MOTOR Y SENSORES
// ================================================================
inline float hill(float x) { return (HILL_A * x * x) / (HILL_B * HILL_B + x * x); }

void setMotor(int ch, int pinA, int pinB, int v) {
    digitalWrite(pinA, v > 0 ? HIGH : LOW); digitalWrite(pinB, v < 0 ? HIGH : LOW); ledcWrite(ch, abs(v));
}

void frenar() { setMotor(CH_M1, M1_A, M1_B, 0); setMotor(CH_M2, M2_A, M2_B, 0); setMotor(CH_M3, M3_A, M3_B, 0); setMotor(CH_M4, M4_A, M4_B, 0); }
void avanzar() { setMotor(CH_M1, M1_A, M1_B, PWM_M1); setMotor(CH_M2, M2_A, M2_B, PWM_M2); setMotor(CH_M3, M3_A, M3_B, PWM_M3); setMotor(CH_M4, M4_A, M4_B, PWM_M4); }
void girar() { setMotor(CH_M1, M1_A, M1_B, PWM_GIRO_M1); setMotor(CH_M2, M2_A, M2_B, PWM_GIRO_M2); setMotor(CH_M3, M3_A, M3_B, PWM_GIRO_M3); setMotor(CH_M4, M4_A, M4_B, PWM_GIRO_M4); }

void acercarse(float d) {
    float ratio = (float)map(constrain((long)d, 6, 200), 6, 200, 40, 150) / 150.0f;
    setMotor(CH_M1, M1_A, M1_B, (int)(PWM_M1 * ratio)); setMotor(CH_M2, M2_A, M2_B, (int)(PWM_M2 * ratio));
    setMotor(CH_M3, M3_A, M3_B, (int)(PWM_M3 * ratio)); setMotor(CH_M4, M4_A, M4_B, (int)(PWM_M4 * ratio));
}

void leerUltrasonidos() {
    auto ping = [](int t, int e) {
        digitalWrite(t, LOW); delayMicroseconds(2); digitalWrite(t, HIGH); delayMicroseconds(10); digitalWrite(t, LOW);
        long dur = pulseIn(e, HIGH, US_TIMEOUT_US);
        return (dur > 0) ? dur * 0.01723f : 999.0f;
    };
    distFront = ping(TRIG_FRONTAL, ECHO_FRONTAL);
    distObj = ping(TRIG_OBJETIVO, ECHO_OBJETIVO);
}

// ================================================================
// IMU Y KALMAN
// ================================================================
bool initIMU() {
    I2C_IMU.beginTransmission(MPU_ADDR); I2C_IMU.write(0x6B); I2C_IMU.write(0x00);
    if (I2C_IMU.endTransmission() != 0) return false;
    delay(100);
    I2C_IMU.beginTransmission(MPU_ADDR); I2C_IMU.write(0x1A); I2C_IMU.write(0x05);
    I2C_IMU.endTransmission();
    return true;
}

bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    I2C_IMU.beginTransmission(MPU_ADDR); I2C_IMU.write(0x3B);
    if (I2C_IMU.endTransmission(false) != 0) return false;
    I2C_IMU.requestFrom(MPU_ADDR, 14);
    if (I2C_IMU.available() < 14) return false;
    auto rd = [&]() -> int16_t { return (I2C_IMU.read() << 8) | I2C_IMU.read(); };
    ax=rd()/4096.0f; ay=rd()/4096.0f; az=rd()/4096.0f;
    rd(); gx=rd()/65.5f; gy=rd()/65.5f; gz=rd()/65.5f;
    return true;
}

void kalman(float &ang, float &unc, float rate, float meas, float dt) {
    ang += dt * rate; unc += dt * dt * 16.0f;
    float g = unc / (unc + 9.0f); ang += g * (meas - ang); unc = (1.0f - g) * unc;
}

// ================================================================
// NEUROCONTROLADOR
// ================================================================
void neuroStep(float imuSig, float dFront, float dObj) {
    float retro = (n[N_OSC_A] + n[N_OSC_B]) * W_RETRO;
    n[N_TERRENO] += (DT/TAU_TERRENO) * (-n[N_TERRENO] + hill(imuSig) + retro);
    float sF = 30.0f / max(dFront, 0.1f) * 100.0f;
    float sO = 30.0f / max(dObj, 0.1f) * 100.0f;
    n[N_PROX_IZQ] += (DT/TAU_PROX) * (-n[N_PROX_IZQ] + hill(sF));
    n[N_PROX_DER] += (DT/TAU_PROX) * (-n[N_PROX_DER] + hill(sO));
    bool obstaculo = (dFront <= DIST_PARADA || dObj <= DIST_PARADA);
    n[N_PARADA] += (DT/TAU_PARADA_T) * (-n[N_PARADA] + (obstaculo ? 100.0f : 0.0f));
    float nTerrNorm = n[N_TERRENO] / (n[N_TERRENO] + 1.0f);
    mem += (DT/TAU_MEM) * (-mem + nTerrNorm * 100.0f);
    n[N_INTEGRADOR] += (DT/TAU_INTEGRADOR) * (-n[N_INTEGRADOR] + n[N_TERRENO]*W_TERRENO + n[N_PROX_IZQ]*W_PROX + n[N_PROX_DER]*W_PROX);
    n[N_OSC_A] += (DT/TAU_OSC_A) * (-n[N_OSC_A] + mem*W_MEM_OSC + n[N_OSC_B]*W_OSC_MUT - n[N_INTEGRADOR]*W_INH_OSC);
    n[N_OSC_B] += (DT/TAU_OSC_B) * (-n[N_OSC_B] + mem*W_MEM_OSC + n[N_OSC_A]*W_OSC_MUT - n[N_INTEGRADOR]*W_INH_OSC);
    bool objEnRango = (dObj > DIST_OBJETIVO && dObj < 200.0f);
    float sObj = objEnRango ? hill(100.0f / max(dObj, 0.1f)) * W_OBJ : 0.0f;
    nAtr += (DT/TAU_ATR) * (-nAtr + sObj);
    nArb += (DT/TAU_ARB) * (-nArb + n[N_INTEGRADOR] * W_EVA + nAtr * W_ATR);
    for (int i=0; i<7; i++) n[i] = constrain(n[i], 0, 100);
    mem=constrain(mem,0,100); nAtr=constrain(nAtr,0,100); nArb=constrain(nArb,0,100);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX_S3, TX_S3);
    
    telMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(tareaSerial2, "TelTask", 4096, NULL, 1, NULL, 0);

    pinMode(M2_B, OUTPUT); digitalWrite(M2_B, LOW);
    int salidas[] = {STBY_IZQ, M1_A, M1_B, M2_A, STBY_DER, M3_A, M3_B, M4_A, M4_B, TRIG_FRONTAL, TRIG_OBJETIVO};
    for (int p : salidas) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

    ledcSetup(CH_M1, 5000, 8); ledcAttachPin(M1_PWM, CH_M1);
    ledcSetup(CH_M2, 5000, 8); ledcAttachPin(M2_PWM, CH_M2);
    ledcSetup(CH_M3, 5000, 8); ledcAttachPin(M3_PWM, CH_M3);
    ledcSetup(CH_M4, 5000, 8); ledcAttachPin(M4_PWM, CH_M4);
    
    digitalWrite(STBY_IZQ, HIGH); digitalWrite(STBY_DER, HIGH);
    pinMode(ECHO_FRONTAL, INPUT); pinMode(ECHO_OBJETIVO, INPUT);

    I2C_IMU.begin(SDA_IMU, SCL_IMU, 400000);
    imuOk = initIMU();
    tIMU = micros();
    estado = RECTO;
    Serial.println(">> SISTEMA OK - TELEMETRIA ACTIVA");
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    static unsigned long tLoop = 0;
    if (millis() - tLoop < 15) return;
    tLoop = millis();

    leerUltrasonidos();

    float imuSig = 0;
    if (imuOk) {
        float ax, ay, az, gx, gy, gz;
        if (readIMU(ax, ay, az, gx, gy, gz)) {
            float dt = (micros() - tIMU) / 1e6f; tIMU = micros();
            float rR = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.3f;
            float rP = -atan2f(ax, sqrtf(ay*ay + az*az)) * 57.3f;
            kalman(roll, rollUnc, gx - gyroOffset[0], rR, dt);
            kalman(pitch, pitchUnc, gy - gyroOffset[1], rP, dt);
            yaw += (gz - gyroOffset[2]) * dt;
            bool terr = (fabsf(roll) > THR_ROLL || fabsf(pitch) > THR_PITCH);
            imuSig = terr ? min(55.0f, fabsf(roll)*2.5f + fabsf(pitch)*1.5f) : 0.0f;
        }
    }

    neuroStep(imuSig, distFront, distObj);

    bool enParada = (n[N_PARADA] > THR_PARADA_N);
    bool terrOn = (fabsf(roll) > THR_ROLL || fabsf(pitch) > THR_PITCH);
    bool meta = (distObj >= DIST_OBJETIVO - TOLERANCIA && distObj <= DIST_OBJETIVO + TOLERANCIA);

    const char* stNombre = "RECTO";

    if (enParada) { frenar(); estado = RECTO; stNombre = "PARADA"; }
    else if (meta) { frenar(); estado = META_OK; stNombre = "META-OK"; }
    else {
        float fEva = (n[N_INTEGRADOR] * W_EVA) / max(nArb, 0.001f);
        float fAtr = (nAtr * W_ATR) / max(nArb, 0.001f);

        if (fEva >= fAtr) {
            if (estado == GIRANDO) {
                if (fabsf(yaw - yawGiroInicio) >= GRADOS_GIRO) { estado = TERRENO_RUGOSO; avanzar(); }
                else girar();
                stNombre = "GIRANDO";
            } else if (terrOn && estado != TERRENO_RUGOSO) {
                yawGiroInicio = yaw; estado = GIRANDO; girar();
                stNombre = "GIRO-INIT";
            } else {
                if (estado == TERRENO_RUGOSO && !terrOn) estado = RECTO;
                avanzar();
                stNombre = (estado == TERRENO_RUGOSO) ? "RUGOSO" : "RECTO";
            }
        } else {
            estado = APROX_OBJETIVO; acercarse(distObj);
            stNombre = "APROX-OBJ";
        }
    }

    // TELEMETRÍA S3
    static unsigned long tTel = 0;
    if (millis() - tTel > 100) {
        tTel = millis();
        if (!telListo) {
            if (xSemaphoreTake(telMutex, 0) == pdTRUE) {
                snprintf(telBuf, sizeof(telBuf),
                    "ST:%s,DF:%.1f,OBJ:%.1f,R:%.1f,P:%.1f,Y:%.1f,N0:%.1f,N1:%.1f,N2:%.1f,N3:%.1f,N4:%.1f,N5:%.1f,N6:%.1f,M:%.1f,A:%.1f,B:%.1f\n",
                    stNombre, distFront, distObj, roll, pitch, yaw,
                    n[N_TERRENO], n[N_PROX_IZQ], n[N_PROX_DER], n[N_PARADA],
                    n[N_INTEGRADOR], n[N_OSC_A], n[N_OSC_B], mem, nAtr, nArb);
                telListo = true;
                xSemaphoreGive(telMutex);
            }
        }
    }
}
