#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================================================================
// DRIVER 1
// STBY → GPIO 27
// PWMA → GPIO 33  (Motor A - Trasera Derecha)
// AIN1 → GPIO 26  AIN2 → GPIO 25
// PWMB → GPIO 13  (Motor B - Delantera Derecha)
// BIN1 → GPIO 14  BIN2 → GPIO 12
// ================================================================
const int STBY1  = 27;
const int MA_PWM = 33, MA_IN1 = 26, MA_IN2 = 25;
const int MB_PWM = 13, MB_IN1 = 14, MB_IN2 = 12;

// ================================================================
// DRIVER 2
// STBY → GPIO 17
// PWMA → GPIO 0   (Motor C - Delantera Izquierda)
// AIN1 → GPIO 16  AIN2 → GPIO 4
// PWMB → GPIO 32  (Motor D - Trasera Izquierda)
// BIN1 → GPIO 5   BIN2 → GPIO 23
// ================================================================
const int STBY2  = 17;
const int MC_PWM =  0, MC_IN1 = 16, MC_IN2 = 4;
const int MD_PWM = 32, MD_IN1 =  5, MD_IN2 = 23;

// ================================================================
// PWM
// ================================================================
const int FREQ = 5000, RES = 8;
const int CH_A = 0, CH_B = 1, CH_C = 2, CH_D = 3;

const int VEL_A = 64;   // Trasera Derecha
const int VEL_B = 55;   // Delantera Derecha
const int VEL_C = 64;   // Delantera Izquierda
const int VEL_D = 29;   // Trasera Izquierda

// ================================================================
// ULTRASONIDOS
// ================================================================
const int TRIG_FRONTAL = 15, ECHO_FRONTAL  = 34;
const int TRIG_OBJETIVO = 2, ECHO_OBJETIVO = 35;
#define US_TIMEOUT_US 5000UL

float distFront = 999.0f;
float distObj   = 999.0f;

void leerUltrasonidos() {
    digitalWrite(TRIG_FRONTAL, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_FRONTAL, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_FRONTAL, LOW);
    long t1 = pulseIn(ECHO_FRONTAL, HIGH, US_TIMEOUT_US);
    if (t1 > 0) distFront = t1 * 0.01723f;

    delayMicroseconds(100);

    digitalWrite(TRIG_OBJETIVO, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_OBJETIVO, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_OBJETIVO, LOW);
    long t2 = pulseIn(ECHO_OBJETIVO, HIGH, US_TIMEOUT_US);
    if (t2 > 0) distObj = t2 * 0.01723f;
}

// ================================================================
// IMU — MPU6050 en I2C bus 1 (SDA=18, SCL=19)
// ================================================================
TwoWire I2C_IMU = TwoWire(1);
const int SDA_IMU = 18, SCL_IMU = 19;
const int MPU_ADDR = 0x68;

float gyroOffset[3] = {0, 0, 0};
float roll = 0, rollUnc = 4.0f;
float pitch = 0, pitchUnc = 4.0f;
float yaw = 0;
unsigned long tIMU = 0;
bool imuOk = false;

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
    rd(); // temperatura — descartar
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
// RED NEURONAL
//
// Neuronas:
//   n[0] N_TERRENO   : responde a IMU (roll/pitch)
//   n[1] N_PROX_IZQ  : responde a distFront
//   n[2] N_PROX_DER  : responde a distObj
//   n[3] N_PARADA    : dispara cuando sensor <= 16 cm
//   n[4] N_INTEGRADOR: integra terreno + proximidad
//   n[5] N_OSC_A     : osciladora A
//   n[6] N_OSC_B     : osciladora B
// ================================================================
float n[7] = {0};
float mem  = 0;
float nAtr = 0;
float nArb = 0;

#define N_TERRENO     0
#define N_PROX_IZQ    1
#define N_PROX_DER    2
#define N_PARADA      3
#define N_INTEGRADOR  4
#define N_OSC_A       5
#define N_OSC_B       6

const float DIST_PARADA_MAX = 14.0f;
const float DIST_OBJETIVO   = 12.0f;
const float TOLERANCIA      =  1.0f;

const float THR_ROLL  = 10.0f;   // umbral IMU roll  [°]
const float THR_PITCH = 10.0f;   // umbral IMU pitch [°]

const float DT              = 10.0f;
const float TAU_TERRENO     = 25.0f;
const float TAU_PROX        = 18.0f;
const float TAU_PARADA_T    =  8.0f;
const float TAU_INTEGRADOR  = 15.0f;
const float TAU_MEM         = 60.0f;
const float TAU_OSC_A       = 12.0f;
const float TAU_OSC_B       = 20.0f;
const float TAU_ATR         = 20.0f;
const float TAU_ARB         = 10.0f;

const float HILL_A = 100.0f;
const float HILL_B = 120.0f;
const float THR_PARADA_N = 60.0f;

const float W_TERRENO  = 0.8f;
const float W_PROX     = 0.4f;
const float W_MEM_OSC  = 0.6f;
const float W_OSC_MUT  = 0.15f;
const float W_INH_OSC  = 0.35f;
const float W_RETRO    = 0.15f;
const float W_OBJ      = 0.9f;
const float W_EVA      = 0.6f;
const float W_ATR      = 0.4f;

inline float hill(float x) {
    return (HILL_A * x * x) / (HILL_B * HILL_B + x * x);
}

void neuroStep(float imuSig, float dFront, float dObj) {
    // N_TERRENO: integra señal IMU con retroalimentación de osciladoras
    float retro = (n[N_OSC_A] + n[N_OSC_B]) * W_RETRO;
    n[N_TERRENO] += (DT / TAU_TERRENO) * (-n[N_TERRENO] + hill(imuSig) + retro);

    // N_PROX_IZQ / N_PROX_DER: señal escalada (30 cm referencia)
    float sF = 30.0f / max(dFront, 0.1f) * 100.0f;
    float sO = 30.0f / max(dObj,   0.1f) * 100.0f;
    n[N_PROX_IZQ] += (DT / TAU_PROX) * (-n[N_PROX_IZQ] + hill(sF));
    n[N_PROX_DER] += (DT / TAU_PROX) * (-n[N_PROX_DER] + hill(sO));

    // N_PARADA: disparo si cualquier sensor <= umbral
    bool obstaculo = (dFront <= DIST_PARADA_MAX || dObj <= DIST_PARADA_MAX);
    n[N_PARADA] += (DT / TAU_PARADA_T) * (-n[N_PARADA] + (obstaculo ? 100.0f : 0.0f));

    // Memoria de terreno (integración lenta)
    float nTerrNorm = n[N_TERRENO] / (n[N_TERRENO] + 1.0f);
    mem += (DT / TAU_MEM) * (-mem + nTerrNorm * 100.0f);

    // N_INTEGRADOR: suma terreno + proximidad
    n[N_INTEGRADOR] += (DT / TAU_INTEGRADOR) * (
        -n[N_INTEGRADOR]
        + n[N_TERRENO]  * W_TERRENO
        + n[N_PROX_IZQ] * W_PROX
        + n[N_PROX_DER] * W_PROX);

    // Osciladoras
    n[N_OSC_A] += (DT / TAU_OSC_A) * (
        -n[N_OSC_A]
        + mem        * W_MEM_OSC
        + n[N_OSC_B] * W_OSC_MUT
        - n[N_INTEGRADOR] * W_INH_OSC);

    n[N_OSC_B] += (DT / TAU_OSC_B) * (
        -n[N_OSC_B]
        + mem        * W_MEM_OSC
        + n[N_OSC_A] * W_OSC_MUT
        - n[N_INTEGRADOR] * W_INH_OSC);

    // Atracción al objetivo
    bool objEnRango = (dObj > DIST_PARADA_MAX && dObj < 200.0f);
    float sObj = objEnRango ? hill(100.0f / max(dObj, 0.1f)) * W_OBJ : 0.0f;
    nAtr += (DT / TAU_ATR) * (-nAtr + sObj);

    // Árbitro
    nArb += (DT / TAU_ARB) * (-nArb + n[N_INTEGRADOR] * W_EVA + nAtr * W_ATR);

    // Clamp [0, 100]
    for (int i = 0; i < 7; i++) n[i] = constrain(n[i], 0.0f, 100.0f);
    mem  = constrain(mem,  0.0f, 100.0f);
    nAtr = constrain(nAtr, 0.0f, 100.0f);
    nArb = constrain(nArb, 0.0f, 100.0f);
}

// ================================================================
// MOTORES
// ================================================================
void controlMotor(int canal, int pin1, int pin2, int vel) {
    digitalWrite(pin1, vel > 0 ? HIGH : LOW);
    digitalWrite(pin2, vel < 0 ? HIGH : LOW);
    ledcWrite(canal, abs(vel));
}

void traseraDerechA(int v)     { controlMotor(CH_A, MA_IN1, MA_IN2, -v); }
void delanteraDerechA(int v)   { controlMotor(CH_B, MB_IN1, MB_IN2,  v); }
void delanteraIzquierda(int v) { controlMotor(CH_C, MC_IN1, MC_IN2, -v); }
void traseraIzquierda(int v)   { controlMotor(CH_D, MD_IN1, MD_IN2, -v); }

void frenarTodos() {
    traseraDerechA(0); delanteraDerechA(0);
    delanteraIzquierda(0); traseraIzquierda(0);
}

void avanzar() {
    traseraDerechA(VEL_A);
    delanteraDerechA(VEL_B);
    delanteraIzquierda(VEL_C);
    traseraIzquierda(VEL_D);
}

void acercarse(float d) {
    float ratio = (float)map(constrain((long)d, 16, 200), 16, 200, 30, 150) / 150.0f;
    traseraDerechA((int)(VEL_A * ratio));
    delanteraDerechA((int)(VEL_B * ratio));
    delanteraIzquierda((int)(VEL_C * ratio));
    traseraIzquierda((int)(VEL_D * ratio));
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== NEUROROBOT INICIANDO ===");

    int salidas[] = {
        STBY1, MA_IN1, MA_IN2, MB_IN1, MB_IN2,
        STBY2, MC_IN1, MC_IN2, MD_IN1, MD_IN2,
        TRIG_FRONTAL, TRIG_OBJETIVO
    };
    for (int p : salidas) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

    pinMode(ECHO_FRONTAL,  INPUT);
    pinMode(ECHO_OBJETIVO, INPUT);

    ledcSetup(CH_A, FREQ, RES); ledcAttachPin(MA_PWM, CH_A);
    ledcSetup(CH_B, FREQ, RES); ledcAttachPin(MB_PWM, CH_B);
    ledcSetup(CH_C, FREQ, RES); ledcAttachPin(MC_PWM, CH_C);
    ledcSetup(CH_D, FREQ, RES); ledcAttachPin(MD_PWM, CH_D);

    ledcWrite(CH_A, 0); ledcWrite(CH_B, 0);
    ledcWrite(CH_C, 0); ledcWrite(CH_D, 0);

    digitalWrite(STBY1, HIGH);
    digitalWrite(STBY2, HIGH);

    // IMU
    I2C_IMU.begin(SDA_IMU, SCL_IMU, 400000);
    imuOk = initIMU();
    Serial.println(imuOk ? "[OK] IMU MPU6050" : "[ERROR] IMU no detectada");
    tIMU = micros();
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    static unsigned long tLoop = 0;
    if (millis() - tLoop < 15) return;
    tLoop = millis();

    leerUltrasonidos();

    // ── IMU ──────────────────────────────────────────────────
    float imuSig = 0.0f;
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

    neuroStep(imuSig, distFront, distObj);

    bool enParada = (n[N_PARADA] > THR_PARADA_N);
    bool metaOk   = (distObj >= DIST_OBJETIVO - TOLERANCIA &&
                     distObj <= DIST_OBJETIVO + TOLERANCIA);

    if (enParada || metaOk) {
        frenarTodos();
    } else {
        float fEva = (n[N_INTEGRADOR] * W_EVA) / max(nArb, 0.001f);
        float fAtr = (nAtr            * W_ATR) / max(nArb, 0.001f);

        if (fAtr > fEva) {
            acercarse(distObj);
        } else {
            avanzar();
        }
    }
    
    Serial.printf("DF:%5.1f OBJ:%5.1f | R:%5.1f P:%5.1f Y:%6.1f | imuSig:%4.1f\n",
        distFront, distObj, roll, pitch, yaw, imuSig);
    }
