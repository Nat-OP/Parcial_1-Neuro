#include <Arduino.h>
#include <Wire.h>

// ================================================================
// DRIVER 1
// ================================================================
const int STBY1 = 27;
const int MA_PWM = 33, MA_IN1 = 26, MA_IN2 = 25;
const int MB_PWM = 13, MB_IN1 = 14, MB_IN2 = 12;

// ================================================================
// DRIVER 2
// ================================================================
const int STBY2 = 17;
const int MC_PWM = 0, MC_IN1 = 16, MC_IN2 = 4;
const int MD_PWM = 32, MD_IN1 = 5, MD_IN2 = 23;

// ================================================================
// CONFIGURACIÓN PWM
// ================================================================
const int FREQ = 5000, RES = 8;
const int CH_A = 0, CH_B = 1, CH_C = 2, CH_D = 3;
const int VEL_A = 64, VEL_B = 64, VEL_C = 64, VEL_D = 32;

// ================================================================
// IMU — MPU6050
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

// 🛑 UMBRAL DE FRENADO
const float THR_INCLINACION = 45.0f;  // Frenar si roll o pitch >= 45°
bool frenadoActivo = false;

// ================================================================
// FUNCIONES DE MOTOR
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

void todosMotores(int dir) {
    controlMotor(CH_A, MA_IN1, MA_IN2, -dir * VEL_A);
    controlMotor(CH_B, MB_IN1, MB_IN2,  dir * VEL_B);
    controlMotor(CH_C, MC_IN1, MC_IN2, -dir * VEL_C);
    controlMotor(CH_D, MD_IN1, MD_IN2, -dir * VEL_D);
}

void frenarTodos() {
    controlMotor(CH_A, MA_IN1, MA_IN2, 0);
    controlMotor(CH_B, MB_IN1, MB_IN2, 0);
    controlMotor(CH_C, MC_IN1, MC_IN2, 0);
    controlMotor(CH_D, MD_IN1, MD_IN2, 0);
}

// ================================================================
// IMU — Funciones del código fuente
// ================================================================
bool initIMU() {
    I2C_IMU.beginTransmission(MPU_ADDR);
    I2C_IMU.write(0x6B);
    I2C_IMU.write(0x00);
    if (I2C_IMU.endTransmission() != 0) return false;
    delay(100);
    I2C_IMU.beginTransmission(MPU_ADDR);
    I2C_IMU.write(0x1A);
    I2C_IMU.write(0x05);
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
    
    auto readInt16 = [&]() -> int16_t {
        return (I2C_IMU.read() << 8) | I2C_IMU.read();
    };
    
    ax = readInt16() / 4096.0f;
    ay = readInt16() / 4096.0f;
    az = readInt16() / 4096.0f;
    readInt16();  // Temperatura
    gx = readInt16() / 65.5f;
    gy = readInt16() / 65.5f;
    gz = readInt16() / 65.5f;
    return true;
}

void kalman(float &ang, float &unc, float rate, float meas, float dt) {
    ang += dt * rate;
    unc += dt * dt * 16.0f;
    float gain = unc / (unc + 9.0f);
    ang += gain * (meas - ang);
    unc = (1.0f - gain) * unc;
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== 🤖 Robot IMU - Prueba 45° ===");

    // Pines motores
    int pinesSalida[] = {
        STBY1, MA_IN1, MA_IN2, MB_IN1, MB_IN2,
        STBY2, MC_IN1, MC_IN2, MD_IN1, MD_IN2
    };
    for (int p : pinesSalida) {
        pinMode(p, OUTPUT);
    }

    // PWM
    ledcSetup(CH_A, FREQ, RES); ledcAttachPin(MA_PWM, CH_A);
    ledcSetup(CH_B, FREQ, RES); ledcAttachPin(MB_PWM, CH_B);
    ledcSetup(CH_C, FREQ, RES); ledcAttachPin(MC_PWM, CH_C);
    ledcSetup(CH_D, FREQ, RES); ledcAttachPin(MD_PWM, CH_D);

    digitalWrite(STBY1, HIGH);
    digitalWrite(STBY2, HIGH);

    // IMU
    Serial.print("🔌 IMU: ");
    I2C_IMU.begin(SDA_IMU, SCL_IMU, 400000);
    imuOk = initIMU();
    Serial.println(imuOk ? "✅ OK" : "❌ ERROR");
    tIMU = micros();

    // Iniciar motores adelante
    todosMotores(1);
    Serial.println("▶️  Motores: ADELANTE");
    Serial.printf("🛑 Frenado automático a %.0f° de inclinación\n\n", THR_INCLINACION);
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    if (imuOk) {
        float ax, ay, az, gx, gy, gz;
        
        if (readIMU(ax, ay, az, gx, gy, gz)) {
            // Calcular dt
            unsigned long now = micros();
            float dt = (now - tIMU) / 1e6f;
            tIMU = now;
            
            // Ángulos raw desde acelerómetro
            float rawRoll  =  atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
            float rawPitch = -atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;
            
            // Filtro Kalman
            kalman(roll,  rollUnc,  gx - gyroOffset[0], rawRoll,  dt);
            kalman(pitch, pitchUnc, gy - gyroOffset[1], rawPitch, dt);
            yaw += (gz - gyroOffset[2]) * dt;
            if (yaw > 180.0f) yaw -= 360.0f;
            if (yaw < -180.0f) yaw += 360.0f;
            
            // 🛑 VERIFICAR INCLINACIÓN
            float inclinacionMax = fmaxf(fabsf(roll), fabsf(pitch));
            
            if (inclinacionMax >= THR_INCLINACION) {
                if (!frenadoActivo) {
                    Serial.printf("\n🛑 FRENADO | Inclinación: %.1f° (Roll: %.1f° | Pitch: %.1f°)\n", 
                                  inclinacionMax, roll, pitch);
                    frenadoActivo = true;
                }
                frenarTodos();
            } else {
                if (frenadoActivo) {
                    Serial.printf("✅ REANUDAR | Inclinación: %.1f°\n", inclinacionMax);
                    frenadoActivo = false;
                }
                todosMotores(1);
            }
            
            // Telemetría (cada 500ms)
            static unsigned long lastPrint = 0;
            if (millis() - lastPrint >= 500) {
                lastPrint = millis();
                Serial.printf("📐 R:%6.1f° P:%6.1f° Y:%6.1f° | Max:%5.1f° | %s\n",
                    roll, pitch, yaw, inclinacionMax,
                    frenadoActivo ? "🛑 FRENADO" : "▶️  ADELANTE");
            }
        }
    } else {
        // IMU no disponible
        static unsigned long lastErr = 0;
        if (millis() - lastErr >= 2000) {
            Serial.println("⚠️  IMU no disponible - verificá conexiones");
            lastErr = millis();
        }
        todosMotores(1);  // Seguir moviendo si no hay IMU
    }
    
    delay(10);
}
