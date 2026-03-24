#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================================================================
// PINOUT Y CALIBRACIÓN DE MOTORES (TU CONFIGURACIÓN EXACTA)
// ================================================================
const int STBY1 = 27, STBY2 = 17;
const int MA_PWM = 33, MA_IN1 = 26, MA_IN2 = 25; // Trasera Derecha
const int MB_PWM = 13, MB_IN1 = 14, MB_IN2 = 12; // Delantera Derecha
const int MC_PWM =  0, MC_IN1 = 16, MC_IN2 =  4; // Delantera Izquierda
const int MD_PWM = 32, MD_IN1 =  5, MD_IN2 = 23; // Trasera Izquierda

const int CH_A = 0, CH_B = 1, CH_C = 2, CH_D = 3;
const int VEL_A = 64, VEL_B = 55, VEL_C = 64, VEL_D = 29; // Tus velocidades

// ================================================================
// SENSORES Y COMUNICACIÓN
// ================================================================
const int TRIG_FRONTAL = 15, ECHO_FRONTAL = 34;
const int TRIG_OBJETIVO = 2, ECHO_OBJETIVO = 35;
#define US_TIMEOUT_US 5000UL

float distFront = 999.0f, distObj = 999.0f;

// SERIAL2 — TELEMETRÍA S3 MINI
#define TX_S3 22
#define RX_S3 21

// ================================================================
// LÓGICA DE MOTORES (COPIADA DE TU CÓDIGO FUNCIONAL)
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
    traseraDerechA(VEL_A); delanteraDerechA(VEL_B);
    delanteraIzquierda(VEL_C); traseraIzquierda(VEL_D);
}

void girar() {
    traseraDerechA(-100); delanteraDerechA(-100);
    delanteraIzquierda(100); traseraIzquierda(100);
}

void acercarse(float d) {
    float ratio = (float)map(constrain((long)d, 16, 200), 16, 200, 30, 150) / 150.0f;
    traseraDerechA((int)(VEL_A * ratio)); delanteraDerechA((int)(VEL_B * ratio));
    delanteraIzquierda((int)(VEL_C * ratio)); traseraIzquierda((int)(VEL_D * ratio));
}

// ================================================================
// IMU, KALMAN Y RED NEURONAL (Mantenemos la estructura anterior)
// ================================================================
TwoWire I2C_IMU = TwoWire(1);
const int SDA_IMU = 18, SCL_IMU = 19, MPU_ADDR = 0x68;
float roll = 0, rollUnc = 4.0f, pitch = 0, pitchUnc = 4.0f, yaw = 0;
float gyroOffset[3] = {0, 0, 0};
unsigned long tIMU = 0;
bool imuOk = false;

float n[7] = {0}, mem = 0, nAtr = 0, nArb = 0;
#define N_TERRENO 0
#define N_PROX_IZQ 1
#define N_PROX_DER 2
#define N_PARADA 3
#define N_INTEGRADOR 4
#define N_OSC_A 5
#define N_OSC_B 6

// Constantes de la Red (Tus parámetros)
const float DIST_PARADA_MAX = 14.0f, DIST_OBJETIVO = 12.0f, TOLERANCIA = 1.0f;
const float THR_ROLL = 10.0f, THR_PITCH = 10.0f, GRADOS_GIRO = 45.0f, DT = 10.0f;
const float TAU_TERRENO=25, TAU_PROX=18, TAU_PARADA_T=8, TAU_INTEGRADOR=15, TAU_MEM=60, TAU_OSC_A=12, TAU_OSC_B=20, TAU_ATR=20, TAU_ARB=10;
const float HILL_A=100.0f, HILL_B=120.0f, THR_PARADA_N=60.0f;
const float W_TERRENO=0.8f, W_PROX=0.4f, W_MEM_OSC=0.6f, W_OSC_MUT=0.15f, W_INH_OSC=0.35f, W_RETRO=0.15f, W_OBJ=0.9f, W_EVA=0.6f, W_ATR=0.4f;

// Telemetría
char telBuf[512]; 
volatile bool telListo = false;
SemaphoreHandle_t telMutex;

void tareaSerial2(void* pvParameters) {
    for (;;) {
        if (telListo && xSemaphoreTake(telMutex, portMAX_DELAY) == pdTRUE) {
            Serial2.print(telBuf); telListo = false;
            xSemaphoreGive(telMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ================================================================
// FUNCIONES DE APOYO
// ================================================================
inline float hill(float x) { return (HILL_A * x * x) / (HILL_B * HILL_B + x * x); }

void leerUltrasonidos() {
    auto p = [](int t, int e) {
        digitalWrite(t, LOW); delayMicroseconds(2); digitalWrite(t, HIGH); delayMicroseconds(10); digitalWrite(t, LOW);
        long dur = pulseIn(e, HIGH, US_TIMEOUT_US);
        return (dur > 0) ? dur * 0.01723f : 999.0f;
    };
    distFront = p(TRIG_FRONTAL, ECHO_FRONTAL);
    delayMicroseconds(100);
    distObj = p(TRIG_OBJETIVO, ECHO_OBJETIVO);
}

void kalman(float &ang, float &unc, float rate, float meas, float dt) {
    ang += dt * rate; unc += dt * dt * 16.0f;
    float g = unc / (unc + 9.0f); ang += g * (meas - ang); unc = (1.0f - g) * unc;
}

void neuroStep(float imuSig, float dFront, float dObj) {
    float retro = (n[N_OSC_A] + n[N_OSC_B]) * W_RETRO;
    n[N_TERRENO] += (DT/TAU_TERRENO) * (-n[N_TERRENO] + hill(imuSig) + retro);
    float sF = 30.0f / max(dFront, 0.1f) * 100.0f;
    float sO = 30.0f / max(dObj, 0.1f) * 100.0f;
    n[N_PROX_IZQ] += (DT/TAU_PROX) * (-n[N_PROX_IZQ] + hill(sF));
    n[N_PROX_DER] += (DT/TAU_PROX) * (-n[N_PROX_DER] + hill(sO));
    bool obstaculo = (dFront <= DIST_PARADA_MAX || dObj <= DIST_PARADA_MAX);
    n[N_PARADA] += (DT/TAU_PARADA_T) * (-n[N_PARADA] + (obstaculo ? 100.0f : 0.0f));
    float nTerrNorm = n[N_TERRENO] / (n[N_TERRENO] + 1.0f);
    mem += (DT/TAU_MEM) * (-mem + nTerrNorm * 100.0f);
    n[N_INTEGRADOR] += (DT/TAU_INTEGRADOR) * (-n[N_INTEGRADOR] + n[N_TERRENO]*W_TERRENO + n[N_PROX_IZQ]*W_PROX + n[N_PROX_DER]*W_PROX);
    n[N_OSC_A] += (DT/TAU_OSC_A) * (-n[N_OSC_A] + mem*W_MEM_OSC + n[N_OSC_B]*W_OSC_MUT - n[N_INTEGRADOR]*W_INH_OSC);
    n[N_OSC_B] += (DT/TAU_OSC_B) * (-n[N_OSC_B] + mem*W_MEM_OSC + n[N_OSC_A]*W_OSC_MUT - n[N_INTEGRADOR]*W_INH_OSC);
    nAtr += (DT/TAU_ATR) * (-nAtr + ((dObj > DIST_PARADA_MAX && dObj < 200.0f) ? hill(100.0f/max(dObj,0.1f))*W_OBJ : 0.0f));
    nArb += (DT/TAU_ARB) * (-nArb + n[N_INTEGRADOR]*W_EVA + nAtr*W_ATR);
    for (int i=0; i<7; i++) n[i] = constrain(n[i], 0, 100);
    mem=constrain(mem,0,100); nAtr=constrain(nAtr,0,100); nArb=constrain(nArb,0,100);
}

// ================================================================
// SETUP Y LOOP
// ================================================================
enum Estado { RECTO, GIRANDO, CRUCERO_RUGOSO, APROX_OBJETIVO };
Estado estado = RECTO;
float yawGiroInicio = 0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX_S3, TX_S3);
    telMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(tareaSerial2, "Tel", 4096, NULL, 1, NULL, 0);

    // Configuración de pines de salida
    int salidas[] = {STBY1, STBY2, MA_IN1, MA_IN2, MB_IN1, MB_IN2, MC_IN1, MC_IN2, MD_IN1, MD_IN2, TRIG_FRONTAL, TRIG_OBJETIVO};
    for (int p : salidas) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    pinMode(ECHO_FRONTAL, INPUT); pinMode(ECHO_OBJETIVO, INPUT);

    // PWM para motores
    ledcSetup(CH_A, 5000, 8); ledcAttachPin(MA_PWM, CH_A);
    ledcSetup(CH_B, 5000, 8); ledcAttachPin(MB_PWM, CH_B);
    ledcSetup(CH_C, 5000, 8); ledcAttachPin(MC_PWM, CH_C);
    ledcSetup(CH_D, 5000, 8); ledcAttachPin(MD_PWM, CH_D);

    digitalWrite(STBY1, HIGH); digitalWrite(STBY2, HIGH);

    // --- CORRECCIÓN IMU ---
    I2C_IMU.begin(SDA_IMU, SCL_IMU, 400000);
    
    // Primero intentamos comunicarnos con la dirección
    I2C_IMU.beginTransmission(MPU_ADDR);
    imuOk = (I2C_IMU.endTransmission() == 0); // Aquí es donde se obtiene el resultado (0 = éxito)

    if(imuOk) { 
        // Si respondió, la despertamos
        I2C_IMU.beginTransmission(MPU_ADDR);
        I2C_IMU.write(0x6B); // Registro de gestión de energía
        I2C_IMU.write(0x00); // 0 para despertar
        I2C_IMU.endTransmission(); 
        Serial.println(">> IMU MPU6050 Detectada y Despierta");
    } else {
        Serial.println(">> ERROR: IMU no encontrada en bus I2C");
    }
    
    tIMU = micros();
}

void loop() {
    static unsigned long tL = 0;
    if (millis() - tL > 200) return;
    tL = millis();
    
    if (!telListo && xSemaphoreTake(telMutex, 0) == pdTRUE) {
        // ... snprintf ...
        telListo = true;
        xSemaphoreGive(telMutex);
    }

    // 1. LEER COMANDOS DE LA S3 MINI (Control remoto)
    if (Serial2.available()) {
        char cmd = Serial2.read();
        if (cmd == 'C') { estado = RECTO; Serial.println(">> MODO: CRUCERO"); }
        if (cmd == 'E') { /* Aquí puedes definir una acción de evitación */ Serial.println(">> MODO: EVITACIÓN"); }
        if (cmd == 'T') { estado = APROX_OBJETIVO; Serial.println(">> MODO: TARGET"); }
    }

    leerUltrasonidos();
    
    // Variable label declarada aquí para que sea visible en todo el loop
    const char* label = "STOP"; 

    float imuSig = 0;
    if (imuOk) {
        float ax, ay, az, gx, gy, gz;
        I2C_IMU.beginTransmission(MPU_ADDR); I2C_IMU.write(0x3B); I2C_IMU.endTransmission(false);
        I2C_IMU.requestFrom(MPU_ADDR, 14);
        if (I2C_IMU.available() >= 14) {
            auto rd = []() -> int16_t { return (I2C_IMU.read() << 8) | I2C_IMU.read(); };
            ax=rd()/4096.0f; ay=rd()/4096.0f; az=rd()/4096.0f; rd();
            gx=rd()/65.5f; gy=rd()/65.5f; gz=rd()/65.5f;
            float dt = (micros() - tIMU) / 1e6f; tIMU = micros();
            kalman(roll, rollUnc, gx, atan2f(ay, sqrtf(ax*ax+az*az))*57.3f, dt);
            kalman(pitch, pitchUnc, gy, -atan2f(ax, sqrtf(ay*ay+az*az))*57.3f, dt);
            yaw += gz * dt;
            if (fabsf(roll)>THR_ROLL || fabsf(pitch)>THR_PITCH) imuSig = min(55.0f, fabsf(roll)*2.5f + fabsf(pitch)*1.5f);
        }
    }

    neuroStep(imuSig, distFront, distObj);

    // 2. LÓGICA DE DECISIÓN Y ASIGNACIÓN DE LABEL
    bool parada = (n[N_PARADA] > THR_PARADA_N);
    bool meta = (distObj >= DIST_OBJETIVO - TOLERANCIA && distObj <= DIST_OBJETIVO + TOLERANCIA);

    if (parada || meta) { 
        frenarTodos(); 
        estado = RECTO; 
        label = meta ? "META" : "STOP"; 
    } 
    else {
        float fEva = (n[N_INTEGRADOR] * W_EVA) / max(nArb, 0.001f);
        float fAtr = (nAtr * W_ATR) / max(nArb, 0.001f);

        if (fEva >= fAtr) {
            if (estado == GIRANDO) {
                if (fabsf(yaw - yawGiroInicio) >= GRADOS_GIRO) { 
                    estado = CRUCERO_RUGOSO; avanzar(); 
                } else girar();
                label = "GIRANDO";
            } else if (imuSig > 0 && estado != CRUCERO_RUGOSO) {
                yawGiroInicio = yaw; estado = GIRANDO; girar(); 
                label = "G-INIT";
            } else {
                if (estado == CRUCERO_RUGOSO && imuSig == 0) estado = RECTO;
                avanzar(); 
                label = (estado == CRUCERO_RUGOSO) ? "RUGOSO" : "RECTO";
            }
        } else {
            estado = APROX_OBJETIVO; 
            acercarse(distObj); 
            label = "APROX";
        }
    }

    // --- TELEMETRÍA CORREGIDA ---
        if (!telListo && xSemaphoreTake(telMutex, 0) == pdTRUE) {
            // IMPORTANTE: El orden de las etiquetas debe coincidir con las variables
            snprintf(telBuf, sizeof(telBuf), 
                "ST:%s,DF:%.1f,OBJ:%.1f,R:%.1f,P:%.1f,Y:%.1f,N0:%.1f,N1:%.1f,N2:%.1f,N3:%.1f,N4:%.1f,N5:%.1f,N6:%.1f,MEM:%.1f,ATR:%.1f,ARB:%.1f\n", 
                label,      // %s
                distFront,  // DF
                distObj,    // OBJ
                roll,       // R
                pitch,      // P
                yaw,        // Y  <-- Asegúrate de que esté aquí
                n[0], n[1], n[2], n[3], n[4], n[5], n[6], // N0 a N6
                mem,        // MEM
                nAtr,       // ATR
                nArb        // ARB
            );
            telListo = true; 
            xSemaphoreGive(telMutex);
        }
    }
