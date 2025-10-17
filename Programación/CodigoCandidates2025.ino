// Librerias
#include <AFMotor.h> // Motores
#include <Wire.h> // Multiplexor
#include <Adafruit_TCS34725.h> // RGB
#include <MPU6050.h> // MPU
#include <PID_v1_bc.h> // Control PID

// Motores
AF_DCMotor MDD(3); // Motor Delantero Derecho
AF_DCMotor MDI(4); // Motor Delantero Izquierdo
AF_DCMotor MTD(2); // Motor Trasero Derecho
AF_DCMotor MTI(1); // Motor Trasero Izquierdo

// Ultrasonicos
#define TRIG_PIN_R 52
#define ECHO_PIN_R 53
#define TRIG_PIN 50
#define ECHO_PIN 51
#define TRIG_PIN_L 25
#define ECHO_PIN_L 24

#define MPX 0x70 // Conexion para el multiplexor
#define MPU_C 5 // Canal del MPU en multiplexor

MPU6050 mpu; // mpu

// Variables útiles para el mpu
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float gradosX = 0, gradosY = 0, gradosZ = 0;
float grados_acumulados_x = 0, grados_acumulados_y = 0, grados_acumulados_z = 0;
long last_time_update = 0;

void setup() {
  // Set up del monitor serial
  Serial.begin(9600);
  Wire.begin();

  // Set up de Motores
  MDD.run(RELEASE);
  MDI.run(RELEASE);
  MTD.run(RELEASE);
  MTI.run(RELEASE);

  // Set up de Ultrasonicos
  pinMode(TRIG_PIN_R, OUTPUT); pinMode(ECHO_PIN_R, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_L, OUTPUT); pinMode(ECHO_PIN_L, INPUT);

  // Set up del giroscopio
  selectChannel(MPU_C);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado");
    calibrarMPU6050();
  } else {
    Serial.println("Error con MPU6050");
  }
}

// Funcion para iniciar mpu en multiplexor
void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MPX);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Funcion para calibrar MPU
void calibrarMPU6050() {
  int16_t gx, gy, gz;
  long gx_sum = 0, gy_sum = 0;
  int muestras = 2000;

  for (int i = 0; i < muestras; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    gx_sum += gx;
    gy_sum += gy;
    delay(2);
  }
  // Promedio de lecturas
  gx_offset = gx_sum / muestras;
  gy_offset = gy_sum / muestras;
  Serial.println("Calibración completa.");
}

// Funcion para obtener cambios en angulos en cada eje
void actualizarAngulo() {
  int16_t gx_raw, gy_raw, gz_raw;
  selectChannel(MPU_C);
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  // Funciones para lecturas precisas en grados/s
  float gx = (gx_raw - gx_offset) / 131.0;
  float gy = (gy_raw - gy_offset) / 131.0;
  float gz = (gz_raw - gz_offset) / 131.0;

  long current_time = millis();
  float dt = (current_time - last_time_update) / 1000.0;
  last_time_update = current_time;

  //Cambio en grados
  float gradosX = gx * dt;
  float gradosY = gy * dt;
  float gradosZ = gz * dt;

  grados_acumulados_x += gradosX;
  grados_acumulados_y += gradosY;
  grados_acumulados_z += gradosZ;
}

// --- FUNCIONES DE MOVIMIENTO CON VELOCIDAD VARIABLE ---

// La función ahora acepta un entero 'velocidad' (0-255)
void adelante(int velocidad) {
  MDD.setSpeed(velocidad); // OPTIMO = con velocidad de 90 , pero hay que calibrarlo un poco más alto para ver si esto lo hace menos exacto porque no sube la rampa de 25 grados
  MDI.setSpeed(velocidad);
  MTD.setSpeed(velocidad);//120
  MTI.setSpeed(velocidad);

  MDD.run(FORWARD);
  MDI.run(FORWARD);
  MTD.run(FORWARD);
  MTI.run(FORWARD);
}

void atras(int velocidad) {
  MDD.setSpeed(velocidad);
  MDI.setSpeed(velocidad);//120
  MTD.setSpeed(velocidad);
  MTI.setSpeed(velocidad);

  MDD.run(BACKWARD);
  MDI.run(BACKWARD);
  MTD.run(BACKWARD);
  MTI.run(BACKWARD);
}

void giro_derecha(int velocidad) {
  MDD.setSpeed(velocidad);//140
  MDI.setSpeed(velocidad);
  MTD.setSpeed(velocidad);
  MTI.setSpeed(velocidad);

  MDD.run(BACKWARD);
  MDI.run(FORWARD);
  MTD.run(BACKWARD);
  MTI.run(FORWARD);
}

void giro_izquierda(int velocidad) {
  MDD.setSpeed(velocidad); //140
  MDI.setSpeed(velocidad);
  MTD.setSpeed(velocidad);
  MTI.setSpeed(velocidad);

  MDD.run(FORWARD);
  MDI.run(BACKWARD);
  MTD.run(FORWARD);
  MTI.run(BACKWARD);
}

void detener() {
  MDD.run(RELEASE);
  MDI.run(RELEASE);
  MTD.run(RELEASE);
  MTI.run(RELEASE);
}

// Funcion para dar vueltas de 90, ahora también pide la velocidad de giro
void girar(float anguloObjetivo, bool derecha, int velocidadGiro) {
  // Reiniciar grados en Z para medir el nuevo giro
  grados_acumulados_z = 0;
  last_time_update = millis(); // Reiniciar el tiempo para un cálculo de ángulo preciso

  // Iniciar el movimiento de giro con la velocidad especificada
  if (derecha) {
    giro_derecha(velocidadGiro);
  } else {
    giro_izquierda(velocidadGiro);
  }

  // Bucle que se ejecuta mientras no se alcance el ángulo objetivo
  while (abs(grados_acumulados_z) < anguloObjetivo) {
    actualizarAngulo(); // Actualiza constantemente el ángulo acumulado
  }

  // Detener los motores una vez que se alcanza el ángulo
  detener();
}


// Funcion ultrasonicos
long obtenerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Distancia en cm
  long duracion = pulseIn(echoPin, HIGH);
  long distancia = duracion * 0.034 / 2;
  return distancia;
}

void loop() {

  // Definir velocidades para tener un control más fácil
  int velocidad = 150;
  int velocidadGiro = 150;

  // Distancias de cada Ultrasonico
  long distF = obtenerDistancia(TRIG_PIN, ECHO_PIN); // Frontral
  long distL = obtenerDistancia(TRIG_PIN_L, ECHO_PIN_L); // Izquierdo
  long distR = obtenerDistancia(TRIG_PIN_R, ECHO_PIN_R); // Derecho

  // Codigo Maze y Pelota
  if ((distF < 8) && (distR < 10)) {
    girar(85, false, velocidadGiro); // Girar a la izquierda
  } else if ((distF < 8) && (distL < 10)) {
    girar(85, true, velocidadGiro);  // Girar a la derecha
  } else if ((distF < 8) && (distL < 10) && (distR < 10)) {
    // Dar media vuelta (dos giros a la derecha)
    girar(85, true, velocidadGiro);
    girar(85, true, velocidadGiro);
  } else if ((distL < 10) && (distR < 10)) {
    adelante(velocidad);
  } else if (distF < 8) {
    girar(85, true, velocidadGiro); // Girar a la derecha
  } else {
    adelante(velocidad);
  }
}
