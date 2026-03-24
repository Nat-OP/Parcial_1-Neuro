#include <Arduino.h>

// --- DRIVER 1 ---
// STBY → GPIO 27
// PWMA → GPIO 33  (Motor A - Trasera Derecha)
// AIN1 → GPIO 26
// AIN2 → GPIO 25
// PWMB → GPIO 13  (Motor B - Delantera Derecha)
// BIN1 → GPIO 14
// BIN2 → GPIO 12

const int STBY1 = 27;

const int MA_PWM = 33;
const int MA_IN1 = 26;
const int MA_IN2 = 25;

const int MB_PWM = 13;
const int MB_IN1 = 14;
const int MB_IN2 = 12;

// --- DRIVER 2 ---
// STBY → GPIO 17
// PWMA → GPIO 0   (Motor C - Delantera Izquierda)
// AIN1 → GPIO 16
// AIN2 → GPIO 4
// PWMB → GPIO 32  (Motor D - Trasera Izquierda)
// BIN1 → GPIO 5
// BIN2 → GPIO 23

const int STBY2 = 17;

const int MC_PWM = 0;
const int MC_IN1 = 16;
const int MC_IN2 = 4;

const int MD_PWM = 32;
const int MD_IN1 = 5;
const int MD_IN2 = 23;

// --- CONFIGURACIÓN PWM ---
const int FREQ = 5000;
const int RES  = 8; // 8 bits → 0 a 255

const int CH_A = 0;
const int CH_B = 1;
const int CH_C = 2;
const int CH_D = 3;

// -----------------------------------------------
// VELOCIDADES INDIVIDUALES (0-255)
// -----------------------------------------------
const int VEL_A = 64; // Trasera Derecha
const int VEL_B = 64; // Delantera Derecha
const int VEL_C = 64; // Delantera Izquierda
const int VEL_D = 32; // Trasera Izquierda

// ================================================================
// 🆕 ULTRASONIDOS - Dos sensores (Frontal + Objetivo)
// ================================================================
// Sensor frontal (obstáculos en la trayectoria)
const int TRIG_FRONTAL  = 15;  // GPIO 15 → TRIG
const int ECHO_FRONTAL  = 34;  // GPIO 34 → ECHO (input-only en ESP32)

// Sensor objetivo (meta o punto de referencia)
const int TRIG_OBJETIVO =  2;  // GPIO 2  → TRIG
const int ECHO_OBJETIVO = 35;  // GPIO 35 → ECHO (input-only en ESP32)

#define US_TIMEOUT_US 5000UL   // Timeout: 5ms ≈ 85cm máx

float distFront = 999.0f;      // Distancia frontal en cm
float distObj   = 999.0f;      // Distancia al objetivo en cm

// 🛑 Umbrales de frenado (configurables por separado)
const float DIST_FRENO_FRONT = 5.0f;  // Frenar si obstáculo frontal < 5 cm
const float DIST_FRENO_OBJ   = 5.0f;  // Frenar si objetivo está demasiado cerca

/**
 * Lee ambos sensores ultrasónicos y actualiza distFront y distObj
 * Basado en la función del código fuente, adaptada para dos sensores
 */
void leerUltrasonidos() {
    // ── Sensor FRONTAL ──────────────────────────────────────
    digitalWrite(TRIG_FRONTAL, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_FRONTAL, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_FRONTAL, LOW);
    
    long t1 = pulseIn(ECHO_FRONTAL, HIGH, US_TIMEOUT_US);
    distFront = (t1 > 0) ? t1 * 0.01723f : 999.0f;
    
    // Pequeña pausa para evitar interferencia entre sensores
    delayMicroseconds(100);
    
    // ── Sensor OBJETIVO ─────────────────────────────────────
    digitalWrite(TRIG_OBJETIVO, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_OBJETIVO, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_OBJETIVO, LOW);
    
    long t2 = pulseIn(ECHO_OBJETIVO, HIGH, US_TIMEOUT_US);
    distObj = (t2 > 0) ? t2 * 0.01723f : 999.0f;
}

// ================================================================
// FUNCIONES DE MOTOR (sin cambios)
// ================================================================
void controlMotor(int canal, int pin1, int pin2, int vel) {
    if (vel > 0) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else if (vel < 0) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    } else {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }
    ledcWrite(canal, abs(vel));
}

void traseraDerechA(int dir)     { controlMotor(CH_A, MA_IN1, MA_IN2, -dir * VEL_A); }
void delanteraDerechA(int dir)   { controlMotor(CH_B, MB_IN1, MB_IN2,  dir * VEL_B); }
void delanteraIzquierda(int dir) { controlMotor(CH_C, MC_IN1, MC_IN2, -dir * VEL_C); }
void traseraIzquierda(int dir)   { controlMotor(CH_D, MD_IN1, MD_IN2, -dir * VEL_D); }

void todosMotores(int dir) {
    traseraDerechA(dir);
    delanteraDerechA(dir);
    delanteraIzquierda(dir);
    traseraIzquierda(dir);
}

void frenarTodos() {
    todosMotores(0);
}

// ================================================================
// 🆕 FUNCIÓN DE SEGURIDAD: verificar ambos sensores
// ================================================================
/**
 * Verifica si hay obstáculo cercano en cualquiera de los dos sensores
 * @return true si hay obstáculo < umbral, false en caso contrario
 */
bool verificarObstaculos() {
    leerUltrasonidos();  // Actualizar ambas lecturas
    
    bool frenoFront = (distFront < DIST_FRENO_FRONT && distFront > 0);
    bool frenoObj   = (distObj   < DIST_FRENO_OBJ   && distObj   > 0);
    
    if (frenoFront || frenoObj) {
        Serial.printf("⚠️  OBSTÁCULO | Front: %.1f cm | Obj: %.1f cm - FRENANDO\n", 
                      distFront, distObj);
        frenarTodos();
        return true;
    }
    return false;
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);

    // Pines de motores + ultrasonidos (TRIG como salida)
    int pinesSalida[] = {
        STBY1, MA_IN1, MA_IN2, MB_IN1, MB_IN2,
        STBY2, MC_IN1, MC_IN2, MD_IN1, MD_IN2,
        TRIG_FRONTAL, TRIG_OBJETIVO  // 🆕 Ambos TRIG como OUTPUT
    };
    for (int p : pinesSalida) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);  // Inicializar en LOW para evitar pulsos espurios
    }
    
    // 🆕 Pines ECHO como entrada (GPIO 34 y 35 son input-only en ESP32)
    pinMode(ECHO_FRONTAL, INPUT);
    pinMode(ECHO_OBJETIVO, INPUT);

    // Configuración PWM
    ledcSetup(CH_A, FREQ, RES); ledcAttachPin(MA_PWM, CH_A);
    ledcSetup(CH_B, FREQ, RES); ledcAttachPin(MB_PWM, CH_B);
    ledcSetup(CH_C, FREQ, RES); ledcAttachPin(MC_PWM, CH_C);
    ledcSetup(CH_D, FREQ, RES); ledcAttachPin(MD_PWM, CH_D);

    // Activar drivers
    digitalWrite(STBY1, HIGH);
    digitalWrite(STBY2, HIGH);

    Serial.println("=== Robot 4 motores + 2 Ultrasonidos listo ===");
    Serial.printf("🛡️  Frenado: Frontal < %.1f cm | Objetivo < %.1f cm\n", 
                  DIST_FRENO_FRONT, DIST_FRENO_OBJ);
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    // 🔄 Verificar obstáculo ANTES de cada movimiento
    if (verificarObstaculos()) {
        delay(300);  // Pausa para estabilizar lecturas
        return;
    }

    // --- Todos adelante 5s ---
    Serial.println(">> Todos adelante");
    todosMotores(1);
    
    // 🔄 Verificar DURANTE el movimiento (cada 400ms para mayor reactividad)
    for (int i = 0; i < 12; i++) {  // 12 × 400ms ≈ 5s
        if (verificarObstaculos()) {
            delay(1000);  // Esperar antes de reintentar
            break;
        }
        delay(400);
    }
    
    frenarTodos();
    delay(300);

    // Verificar antes de cambiar dirección
    if (verificarObstaculos()) {
        delay(1000);
        return;
    }

    // --- Todos atrás 5s ---
    Serial.println("<< Todos atrás");
    todosMotores(-1);
    
    for (int i = 0; i < 12; i++) {
        if (verificarObstaculos()) {
            delay(1000);
            break;
        }
        delay(400);
    }
    
    frenarTodos();
    delay(300);
    
    // 📊 Telemetría en Serial Monitor (cada ciclo)
    Serial.printf("📏 Front: %.1f cm | Obj: %.1f cm\n", distFront, distObj);
}
