// - - LIBRERIAS Y DECLARACIONES NECESARIAS PARA FUNCIONAR - - //
// Librerias
#include <AFMotor.h> // Motores
#include <Wire.h> // Multiplexor
#include <Adafruit_TCS34725.h> // RGB
#include <MPU6050.h> // MPU
#include <Servo.h> // Servo
#include <LiquidCrystal_I2C.h> // Pantalla LCD

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

#define LCD_ADDR 0x27 // Conexion para pantalla
#define LCD_C 1       // Canal de la pantalla LCD
#define RGB_C 4       // Canal del Sensor RGB en multiplexor
#define MPU_C 2      // Canal del MPU en multiplexor
#define GARRA_P 10    // Pin de la garra
#define IR_PIN 29 // Pin del sensor infrarrojo

MPU6050 mpu; // MPU
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X); // RGB
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2); // Pantalla LCD
Servo garra;

// Variables útiles para el mpu
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float grados_acumulados_x = 0, grados_acumulados_y = 0, grados_acumulados_z = 0;
long last_time_update = 0;

// Estado del Maze
bool mazeIniciado = false;

// - - SET UP DE LOS COMPONENTES - - //
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Motores
  MDD.run(RELEASE);
  MDI.run(RELEASE);
  MTD.run(RELEASE);
  MTI.run(RELEASE);

  // Ultrasonicos
  pinMode(TRIG_PIN_R, OUTPUT); pinMode(ECHO_PIN_R, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_L, OUTPUT); pinMode(ECHO_PIN_L, INPUT);

  // MPU
  selectChannel(MPU_C);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado");
    calibrarMPU6050();
  } else {
    Serial.println("Error con MPU6050");
  }

  // RGB
  selectChannel(RGB_C);
  if (tcs.begin()) Serial.println("Sensor TCS34725 encontrado.");
  else { Serial.println("No se encontró el sensor TCS34725."); while (1); }

  // LCD
  selectChannel(LCD_C);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Robot Candidates");
  delay(1000);

  // Servo
  garra.attach(GARRA_P);
  garra.write(140);

  // Infrarojo
  pinMode(IR_PIN, INPUT);
}

// - - FUNCIONES DEL MULTIPLEXOR - - //
void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MPX);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// - - FUNCIONES DE MOVIMIENTO - - //
void adelante(int velocidad) {
  MDD.setSpeed(velocidad); MDI.setSpeed(velocidad);
  MTD.setSpeed(velocidad); MTI.setSpeed(velocidad);
  MDD.run(FORWARD); MDI.run(FORWARD);
  MTD.run(FORWARD); MTI.run(FORWARD);
}

void atras(int velocidad) {
  MDD.setSpeed(velocidad); MDI.setSpeed(velocidad);
  MTD.setSpeed(velocidad); MTI.setSpeed(velocidad);
  MDD.run(BACKWARD); MDI.run(BACKWARD);
  MTD.run(BACKWARD); MTI.run(BACKWARD);
}

void giro_derecha(int velocidad) {
  MDD.setSpeed(velocidad); MDI.setSpeed(velocidad);
  MTD.setSpeed(velocidad); MTI.setSpeed(velocidad);
  MDD.run(BACKWARD); MDI.run(FORWARD);
  MTD.run(BACKWARD); MTI.run(FORWARD);
}

void giro_izquierda(int velocidad) {
  MDD.setSpeed(velocidad); MDI.setSpeed(velocidad);
  MTD.setSpeed(velocidad); MTI.setSpeed(velocidad);
  MDD.run(FORWARD); MDI.run(BACKWARD);
  MTD.run(FORWARD); MTI.run(BACKWARD);
}

void detener() {
  MDD.run(RELEASE);
  MDI.run(RELEASE);
  MTD.run(RELEASE);
  MTI.run(RELEASE);
}

// - - FUNCIONES DE SENSORES - - //
long obtenerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(1);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH);
  long distancia = duracion * 0.034 / 2;
  return distancia;
}

// RGB
String Color() {
  uint16_t r, g, b, c;
  selectChannel(RGB_C);
  tcs.getRawData(&r, &g, &b, &c);
  if ((r >= 11 && r <= 25) && (g >= 21 && g <= 35) && (b >= 6 && b <= 20)) return "Verde";
  else if ((r >= 44 && r <= 58) && (g >= 7 && g <= 21) && (b >= 5 && b <= 19)) return "Rojo";
  else if ((r >= 6 && r <= 22) && (g >= 14 && g <= 30) && (b >= 18 && b <= 34)) return "Azul";
  else if ((r >= 100 && r <= 150) && (g >= 80 && g <= 120) && (b >= 15 && b <= 55)) return "Amarillo";
  else if ((r >= 73 && r <= 95) && (g >= 26 && g <= 40) && (b >= 22 && b <= 36)) return "Rosa";
  else if ((r >= 4 && r <= 18) && (g >= 2 && g <= 16) && (b >= 0 && b <= 13)) return "Negro";
  else if (r > 150 && g > 150 && b > 100) return "Blanco";
  else return "---";
}

// - - MPU - - //
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
  gx_offset = gx_sum / muestras;
  gy_offset = gy_sum / muestras;
  Serial.println("Calibración completa.");
}


/*
void actualizarAngulo() {
  int16_t gx_raw, gy_raw, gz_raw;
  selectChannel(MPU_C);
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  float gx = (gx_raw - gx_offset) / 131.0;
  float gy = (gy_raw - gy_offset) / 131.0;
  float gz = (gz_raw) / 131.0;

  long current_time = millis();
  float dt = (current_time - last_time_update) / 1000.0;
  last_time_update = current_time;

  grados_acumulados_z += gz * dt;
}

void girar(float anguloObjetivo, bool derecha, int velocidadGiro) {
  grados_acumulados_z = 0;
  last_time_update = millis();

  if (derecha) giro_derecha(velocidadGiro);
  else giro_izquierda(velocidadGiro);

  while (abs(grados_acumulados_z) < anguloObjetivo) actualizarAngulo();
  detener();
}*/

// LCD
void actualizarLCD(long distF, String colorDetectado) {
  selectChannel(LCD_C);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist F: ");
  lcd.print(distF);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  lcd.print("Color: ");
  lcd.print(colorDetectado);
}

// - - LOOP PARA MAZE - - //
/*bool giroDerechaSiguiente = false; 

void loop() {
  int velocidad = 150;
  int velocidadGiro = 150;

  static bool mazeIniciado = false;
  static bool regreso = false;

  long distF = obtenerDistancia(TRIG_PIN, ECHO_PIN);
  long distL = obtenerDistancia(TRIG_PIN_L, ECHO_PIN_L);
  long distR = obtenerDistancia(TRIG_PIN_R, ECHO_PIN_R);

  String color = Color();
  actualizarLCD(distF, color);

  // Empezar si detecta color verde
  if (!mazeIniciado && color == "Verde") {
    mazeIniciado = true;
    Serial.println("¡Color verde detectado! Iniciando maze...");
  }

  // Si no detecta color verde, no se mueve
  if (!mazeIniciado) {
    detener();
    return;
  }

  // Retroceder y girar a la derecha si detecta color negro
  if (color == "Negro") {
    Serial.println("Color negro detectado - retrocediendo y girando derecha");
    detener();
    atras(150);
    delay(100);
    detener();
    giro_izquierda(velocidadGiro);
    delay(1550);
    return;
  }

  // Avanzar un momento y detenerse si detecta color rojo
  if (color == "Rojo") {
    Serial.println("Color rojo detectado - avanzando y deteniéndose");
    adelante(150);
    delay(250);
    detener();
    regreso = true; // Se activa siguiente parte del código
  }

  if (regreso) {
    adelante(150);
    if (color == "Verde") {
      Serial.println("Color verde en regreso — sigue 250ms y cambia a reversa");
      adelante(150);
      delay(250);
      atras(150);
    }
    if (color == "Rojo") {
      Serial.println("Color rojo encontrado otra vez, detener");
      detener();
      regreso = false;
      mazeIniciado = false;
      return;
    }
  }

  // Movimiento del robot con base en los ultrasonicos
  if ((distF < 12) && (distR < 12)) {
    giro_izquierda(velocidadGiro);
  }
  else if ((distF < 12) && (distL < 12)) {
    giro_derecha(velocidadGiro);
    delay(1500);
  }
  else if ((distF < 12) && (distL < 12) && (distR < 12)) {
      atras(velocidad);
      delay(1250);
      giro_derecha(velocidadGiro);
      delay(1500);
  }
   else if (distF < 12) {
    detener();
    delay(200);

    // Compuerta logica para evitar caer en loops
    if (giroDerechaSiguiente) {
      Serial.println("Obstáculo al frente — girando DERECHA");
      giro_derecha(velocidadGiro);
      delay(1500);
    } else {
      Serial.println("Obstáculo al frente — girando IZQUIERDA");
      giro_izquierda(velocidadGiro);
      delay(1500);
    }

    // Alternar el sentido para la próxima vez
    giroDerechaSiguiente = !giroDerechaSiguiente;
  }
  //else if ((distR > 15) && (distL > 15)) {
    //giro_izquierda(velocidadGiro);
  //}
  else {
    adelante(velocidad);
  }
} */


// - - LOOP PARA PELOTA - - //

void loop() {

  int velocidad = 60;
  int velocidadGiro = 130;
  static bool recorridoIniciado = false;
  static bool tienePelota = false;
  static bool puedeLeerBandera = true;

  long distF = obtenerDistancia(TRIG_PIN, ECHO_PIN);
  long distL = obtenerDistancia(TRIG_PIN_L, ECHO_PIN_L);
  long distR = obtenerDistancia(TRIG_PIN_R, ECHO_PIN_R);
  String color = Color();
  actualizarLCD(distF, color);

  int irValor = digitalRead(IR_PIN);

  // Empezar si detecta color verde
  if (!recorridoIniciado && color == "Verde") {
    recorridoIniciado = true;
    Serial.println("Inicio detectado (verde) - comenzando recorrido");
    detener();
    delay(500);
    adelante(velocidad);
  }

  // Compuerta logica para iniciar o terminar recorridos
  if (!recorridoIniciado) {
    detener();
    return;
  }

  // Agarrar pelota si el infrarrojo se enciende
  if (irValor == LOW && !tienePelota) {
    detener();
    delay(1000);
    Serial.println("Infrarrojo activado - cerrando garra");
    garra.write(85);
    delay(700);
    tienePelota = true;
    adelante(velocidad);
  }

  // Compuerta lógica para evitar que la garra se active multiples veces
  if (color == "Blanco") {
    if (!puedeLeerBandera) {
        Serial.println("Blanco detectado. Listo para la siguiente bandera.");
        puedeLeerBandera = true;
    }
  }

  if (tienePelota && puedeLeerBandera) {
    if (color == "Azul") {
      Serial.println("Bandera Azul detectada - Soltando pelota");
      detener();
      delay(2000);
      garra.write(140);
      atras(velocidad);
      delay(750);
      adelante(velocidad);
      delay(750);
      garra.write(85);   
      delay(1000);
      adelante(velocidad); 
      puedeLeerBandera = false; 
    }
    else if (color == "Amarillo") {
      Serial.println("Bandera Amarilla detectada - Pateando pelota");
      detener();
      delay(1000);
      garra.write(140);
      delay(500);
      adelante(velocidad); 
      delay(500);           
      detener();
      delay(6000);
      adelante(velocidad);
      tienePelota = false; 
      
      puedeLeerBandera = false;
    }
  }

  // Agarrar pelota si el infrarrojo se enciende
  if (irValor == LOW && !tienePelota) {
    detener();
    delay(1000);
    Serial.println("Infrarrojo activado - cerrando garra");
    garra.write(85);
    delay(700);
    tienePelota = true;
    adelante(velocidad);
    delay(500);
  }

  // Terminar si detecta color rojo
  if (color == "Rojo") {
    Serial.println("Fin detectado (rojo) - Deteniendo robot");
    detener();
    recorridoIniciado = false;
    tienePelota = false;
  }

  // Movimiento del robot con base en los ultrasonicos
  if ((distF < 12) && (distL < 12)) {
    giro_derecha(velocidadGiro);
    delay(1550);
  }
  else if ((distF < 12) && (distR < 12)) {
    giro_izquierda(velocidadGiro);
    delay(1550);
  }
  else {
    adelante(velocidad);
  }
}

