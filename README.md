# 🧠 Parcial 1 - Neurocontrol

## 📌 About

Desarrollo de un robot diferencial con control neuronal para navegación autónoma. Incluye detección de terreno irregular (IMU), aproximación a objetivos mediante sensores de percepción y una arquitectura con memoria para arbitraje de comportamientos. Incluye GUI de monitoreo en tiempo real y validación experimental.

---

## 📖 Descripción

Este proyecto corresponde al **Primer Parcial de Neurocontrol 2026**, enfocado en la implementación de arquitecturas neuronales para resolver problemas de percepción, decisión y control en un robot diferencial.

El sistema integra múltiples comportamientos autónomos, combinando técnicas de control neuronal con validación experimental.

---

## 🎯 Objetivos

* Implementar sistemas neuronales para toma de decisiones
* Desarrollar control neuromotor autónomo
* Integrar comportamientos mediante arquitecturas con memoria
* Validar experimentalmente los resultados

---

## ⚙️ Funcionalidades principales

### 🟠 Detección de terreno

* Uso de sensor IMU
* Clasificación neuronal de terreno (plano vs irregular)
* Respuesta autónoma (giro + trayectoria rectilínea)

### 🔵 Aproximación a objetivo

* Detección de estímulo mediante sensores
* Navegación autónoma
* Detención precisa a 5 cm del objetivo

### 🟢 Integración con memoria

* Resolución de conflicto: evasión vs atracción
* Modulación del comportamiento (temerario / cauteloso)
* Arquitectura neuronal con memoria

---

## 🖥️ Interfaz de visualización

* Monitoreo en tiempo real de actividad neuronal
* Baja latencia (< 2 segundos)
* Visualización de estados internos del sistema

---

## 📂 Estructura del repositorio

```bash
Parcial_1-Neuro/
├── README.md
└── parcial1-neuro/
    ├── ROBOT_STL/              # Modelos 3D del robot
    ├── include/                # Archivos de cabecera
    ├── lib/                    # Librerías
    ├── src/                    # Código principal
    │   └── main.cpp
    ├── test/                   # Pruebas, validación y experimentos
    └── platformio.ini          # Configuración del proyecto
```

---

## 🛠️ Tecnologías utilizadas

* C++ (ESP32 / control del robot)
* Python (interfaz y pruebas)
* PlatformIO
* Sensores: IMU, ultrasonido / distancia
* Comunicación: WiFi / UDP / ESP-NOW

---

## 📊 Resultados

El sistema demostró:

* Detección efectiva de cambios de terreno
* Navegación autónoma precisa hacia objetivos
* Capacidad de adaptación mediante memoria neuronal
* Monitoreo en tiempo real del estado interno

---

## 🚀 Ejecución

1. Clonar el repositorio:

```bash
git clone https://github.com/Nat-OP/Parcial_1-Neuro.git
```

2. Compilar y cargar el código en el robot (PlatformIO)

3. Ejecutar la interfaz en Python para monitoreo

---

## 👥 Autores

* [Tu nombre]
* Equipo de trabajo - Neurocontrol 2026

---

## 📄 Notas

Este proyecto fue desarrollado con fines académicos.
Incluye múltiples iteraciones experimentales documentadas en la carpeta `/test`.

---
