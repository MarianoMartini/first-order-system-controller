#include <Arduino.h>
// ============================================================================
// DEFINICIÓN DE PINES
// ============================================================================
const int ledPWM  = 3;   // Pin PWM (salida del sistema mapeada)
const int ledStat = 4;   // Pin para LED de estado (parpadeo por ciclo)

// ============================================================================
// PARÁMETROS DEL SISTEMA (Planta de 1er Orden)
// ============================================================================
const double k   = 0.8;  // Ganancia estática
const double tau = 10.0; // Constante de tiempo [s]
const double T   = 0.2;  // Período de muestreo [s]
double alpha     = 0.0;  // Factor de discretización: exp(-T / tau)

// ============================================================================
// PARÁMETROS DEL CONTROLADOR PID
// ============================================================================
const double Kc = 2.0;   // Ganancia proporcional
const double Ti = 1.0;   // Tiempo integral [s]
const double Td = 0.01;  // Tiempo derivativo [s]

// Coeficientes del PID discreto
double k1 = 0.0;
double k2 = 0.0;
double k3 = 0.0;

// ============================================================================
// VARIABLES DE ESTADO Y SEÑALES
// ============================================================================
double r = 0.0;                          // Referencia / Setpoint
double y = 0.0, y_1 = 0.0;               // Salida del sistema: actual y(k), previa y(k-1)
double u = 0.0, u_1 = 0.0, u_2 = 0.0;    // Señal de control: u(k), u(k-1), u(k-2)
double e = 0.0, e_1 = 0.0, e_2 = 0.0;    // Error: e(k), e(k-1), e(k-2)

int  out    = 0;                         // Salida mapeada para PWM (0 - 255)
bool estado = false;                     // Estado del LED testigo

// ============================================================================
// DECLARACIÓN DE FUNCIONES AUXILIARES
// ============================================================================
void calcularParametros();
void reiniciarVariables();
void procesarComandosSerial();

// ============================================================================
// CONFIGURACIÓN INICIAL (setup)
// ============================================================================
void setup() {
  pinMode(ledPWM, OUTPUT);
  pinMode(ledStat, OUTPUT);

  Serial.begin(9600);
  delay(1500);
  Serial.println("Iniciando...");

  // Inicialización de parámetros matemáticos
  calcularParametros();

  // Imprimir parámetros calculados y encabezado de telemetría
  Serial.print(alpha);
  Serial.print(" , ");
  Serial.print(k1);
  Serial.print(" , ");
  Serial.print(k2);
  Serial.print(" , ");
  Serial.println(k3);
  Serial.println("r ;      u ;      y");
}

// ============================================================================
// BUCLE PRINCIPAL (loop)
// ============================================================================
void loop() {
  // 1. Comprobar comandos recibidos por el puerto serie
  procesarComandosSerial();

  // 2. Simulación de la planta de 1er orden discretizada:
  //    y(k) = u(k-1) * (1 - alpha) * k + y(k-1) * alpha
  y = u_1 * (1.0 - alpha) * k + y_1 * alpha;

  // 3. Cálculo del error actual
  e = r - y;

  // 4. Ley de control PID en diferencias (forma de velocidad / recurrente):
  //    u(k) = u(k-1) + k1*e(k) + k2*e(k-1) + k3*e(k-2)
  u = u_1 + k1 * e + k2 * e_1 + k3 * e_2;

  // 5. Envío de datos por el puerto serie
  Serial.print(r);
  Serial.print(" ; ");
  Serial.print(u);
  Serial.print(" ; ");
  Serial.println(y);

  // 6. Actualización de salida PWM
  out = map((long)y, 0, 100, 0, 255);
  analogWrite(ledPWM, constrain(out, 0, 255));

  // 7. Toggle del LED de estado para indicar ciclo de ejecución
  estado = !estado;
  digitalWrite(ledStat, estado ? HIGH : LOW);

  // 8. Retardo correspondiente al período de muestreo
  delay(T * 1000);

  // 9. Actualización de estados pasados
  y_1 = y;
  u_2 = u_1;
  u_1 = u;
  e_2 = e_1;
  e_1 = e;
}

// ============================================================================
// DEFINICIÓN DE FUNCIONES AUXILIARES
// ============================================================================

void calcularParametros() {
  alpha = exp(-T / tau);
  k1 = Kc + (Kc * T / Ti) + (Kc * Td / T);
  k2 = -Kc - (2.0 * Kc * Td / T);
  k3 = Kc * Td / T;
}

void reiniciarVariables() {
  u   = 0.0;
  u_1 = 0.0;
  u_2 = 0.0;
  y   = 0.0;
  y_1 = 0.0;
  e   = 0.0;
  e_1 = 0.0;
  e_2 = 0.0;
  r   = 10.0;

  Serial.println("INICIO");
  Serial.println("r ;      u ;      y");
  Serial.flush();
}

void procesarComandosSerial() {
  if (Serial.available() > 0) {
    char ent = Serial.read();

    if (ent == 'r') {
      reiniciarVariables();
    } else if (ent != '\r' && ent != '\n') {
      Serial.println("Entrada no identificada");
    }
  }
}