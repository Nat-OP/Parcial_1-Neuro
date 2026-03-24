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
// 🆕 ULTRASONIDOS - Integrado del código fuente
// ================================================================
const int TRIG_FRONTAL  = 15;  // GPIO 15 → TRIG
const int ECHO_FRONTAL  = 34;  // GPIO 34 → ECHO (input-only en ESP32)
#define US_TIMEOUT_US 5000UL   // Timeout para pulseIn: 5ms ≈ 85cm máx

float distFront = 999.0f;       // Distancia frontal en cm (inicialmente "infinito")
const float DISTANCIA_FRENO = 5.0f;  // 🛑 Umbral de frenado: 5 cm

/**
 * Lee el sensor ultrasónico frontal y actualiza distFront
 * Basado en la función del código fuente, simplificada para un solo sensor
 */
void leerUltrasonidos() {
    // Pulso de trigger: 10µs en HIGH
    digitalWrite(TRIG_FRONTAL, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_FRONTAL, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_FRONTAL, LOW);
    
    // Medir tiempo de eco con timeout
    long duration = pulseIn(ECHO_FRONTAL, HIGH, US_TIMEOUT_US);
    
    // Calcular distancia: velocidad del sonido ≈ 343 m/s → 0.0343 cm/µs
    // Como el sonido va y vuelve: distancia = (tiempo * 0.0343) / 2 ≈ tiempo * 0.01723
    if (duration > 0) {
        distFront = duration * 0.01723f;
    } else {
        distFront = 999.0f;  // Sin lectura válida → distancia "infinita"
    }
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
// 🆕 FUNCIÓN DE SEGURIDAD: verificar obstáculo
// ================================================================
/**
 * Verifica si hay obstáculo cercano y frena si es necesario
 * @return true si hay obstáculo < DISTANCIA_FRENO, false en caso contrario
 */
bool verificarObstaculo() {
    leerUltrasonidos();  // Actualizar lectura del sensor
    
    // 🛑 Frenar si hay obstáculo a menos de 5 cm
    if (distFront < DISTANCIA_FRENO && distFront > 0) {
        Serial.printf("⚠️  OBSTÁCULO: %.1f cm - FRENANDO\n", distFront);
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

    // Pines de motores
    int pinesSalida[] = {
        STBY1, MA_IN1, MA_IN2, MB_IN1, MB_IN2,
        STBY2, MC_IN1, MC_IN2, MD_IN1, MD_IN2,
        TRIG_FRONTAL  // 🆕 Pin TRIG como salida
    };
    for (int p : pinesSalida) {
        pinMode(p, OUTPUT);
    }
    
    // 🆕 Pin ECHO como entrada (GPIO 34 es input-only en ESP32)
    pinMode(ECHO_FRONTAL, INPUT);

    // Configuración PWM
    ledcSetup(CH_A, FREQ, RES); ledcAttachPin(MA_PWM, CH_A);
    ledcSetup(CH_B, FREQ, RES); ledcAttachPin(MB_PWM, CH_B);
    ledcSetup(CH_C, FREQ, RES); ledcAttachPin(MC_PWM, CH_C);
    ledcSetup(CH_D, FREQ, RES); ledcAttachPin(MD_PWM, CH_D);

    // Activar drivers
    digitalWrite(STBY1, HIGH);
    digitalWrite(STBY2, HIGH);
    
    // 🆕 Inicializar trigger en LOW
    digitalWrite(TRIG_FRONTAL, LOW);

    Serial.println("=== Robot 4 motores + Ultrasonido listo ===");
    Serial.printf("🛡️  Frenado automático activado a < %.1f cm\n", DISTANCIA_FRENO);
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    // 🔄 Verificar obstáculo ANTES de cada movimiento
    // Esto asegura que el robot frene inmediatamente si detecta algo
    if (verificarObstaculo()) {
        delay(200);  // Pequeña pausa para evitar lecturas redundantes
        return;      // Salir del loop sin ejecutar movimientos
    }

    // --- Todos adelante 5s ---
    Serial.println(">> Todos adelante");
    todosMotores(1);
    
    // 🔄 Verificar obstáculo DURANTE el movimiento (cada 500ms)
    for (int i = 0; i < 10; i++) {  // 10 × 500ms = 5s
        if (verificarObstaculo()) {
            delay(1000);  // Esperar 1s antes de reintentar
            break;        // Salir del bucle de movimiento
        }
        delay(500);
    }
    
    frenarTodos();
    delay(500);

    // Verificar nuevamente antes de cambiar dirección
    if (verificarObstaculo()) {
        delay(1000);
        return;
    }

    // --- Todos atrás 5s ---
    Serial.println("<< Todos atrás");
    todosMotores(-1);
    
    // 🔄 Verificar obstáculo DURANTE el movimiento hacia atrás
    for (int i = 0; i < 10; i++) {
        if (verificarObstaculo()) {
            delay(1000);
            break;
        }
        delay(500);
    }
    
    frenarTodos();
    delay(500);
    
    // 📊 Telemetría opcional (descomentar si quieres ver distancias)
    // Serial.printf("📏 Distancia frontal: %.1f cm\n", distFront);
}
