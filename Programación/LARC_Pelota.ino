// - - LIBRERIAS Y DECLARACIONES NECESARIAS PARA FUNCIONAR - - //

// Librerias
#include <AFMotor.h> // Motores
#include <Wire.h> // Multiplexor
#include <Adafruit_TCS34725.h> // RGB
#include <Adafruit_BNO08x.h> // BNO
#include <Servo.h> // Servo
#include <LiquidCrystal_I2C.h> // LCD

// Motores
AF_DCMotor MDD(3);  // Motor delantero derecho
AF_DCMotor MDI(4); // Motor delantero izquierdo
AF_DCMotor MTD(2); // Motor trasero derecho
AF_DCMotor MTI(1); // Motor trasero izquierdo

// Ultrasonicos
#define TRIG_PIN_R 52
#define ECHO_PIN_R 53
#define TRIG_PIN 50
#define ECHO_PIN 51
#define TRIG_PIN_L 25
#define ECHO_PIN_L 24

// Multiplexor y arduino
#define MPX 0x70
#define LCD_C 1
#define RGB_C 4
#define BNO_C 5
#define LCD_ADDR 0x27
#define GARRA_P 10
#define IR_PIN 29

// Objetos
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
Servo garra;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// Variables BNO
float yawInicial = 0;
float yawActual = 0;

// Variables generales
bool mazeIniciado = false;
static bool recorridoIniciado = false;
static bool tienePelota = false;
static bool puedeLeerBandera = true;

// - - FUNCIONES MULTIPLEXOR - - //
void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MPX);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// - - FUNCIONES MOVIMIENTO - - //
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

// - - FUNCIONES SENSORES - - //
// Ultrasonicos
long obtenerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(1);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH);
  return duracion * 0.034 / 2;
}

// RGB
String Color() {
  uint16_t r, g, b, c;
  selectChannel(RGB_C);
  tcs.getRawData(&r, &g, &b, &c);
  if ((r >= 11 && r <= 25) && (g >= 21 && g <= 35) && (b >= 6 && b <= 20)) return "Verde";
  else if ((r >= 55 && r <= 85) && (g >= 7 && g <= 24) && (b >= 5 && b <= 23)) return "Rojo";
  else if ((r >= 6 && r <= 22) && (g >= 14 && g <= 30) && (b >= 18 && b <= 34)) return "Azul";
  else if ((r >= 100 && r <= 150) && (g >= 80 && g <= 120) && (b >= 15 && b <= 55)) return "Amarillo";
  else if ((r >= 73 && r <= 95) && (g >= 26 && g <= 40) && (b >= 22 && b <= 36)) return "Rosa";
  else if ((r >= 4 && r <= 18) && (g >= 2 && g <= 16) && (b >= 0 && b <= 13)) return "Negro";
  else if (r > 150 && g > 150 && b > 80) return "Blanco";
  else return "---";
}

// BNO obtener grados
float obtenerYaw() {
  selectChannel(BNO_C);
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;
      float yaw = atan2(2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / PI;
      if (yaw < 0) yaw += 360.0;
      yawActual = yaw;
      return yawActual;
    }
  }
  return yawActual;
}

// BNO girar con precision
void girar(float angulo, bool derecha, int velocidadGiro) {
  selectChannel(BNO_C);
  float yawInicio = obtenerYaw();
  float objetivo;

  if (derecha) {
    objetivo = yawInicio - angulo;
    if (objetivo < 0) {
      objetivo = 360 - fabs(objetivo);
    }
  } else {
    objetivo = yawInicio + angulo;
    if (objetivo >= 360) {
      objetivo = objetivo - 360;
    }
  }

  Serial.print("Inicio: "); Serial.println(yawInicio);
  Serial.print("Objetivo: "); Serial.println(objetivo);

  while (true) {
    float yaw = obtenerYaw();
    float error = fabs(yaw - objetivo);

    // Corrección de salto 360° → 0°
    if (error > 180) error = 360 - error;

    if (error <= 5) break;

    if (derecha) giro_derecha(velocidadGiro);
    else giro_izquierda(velocidadGiro);
  }

  detener();
  delay(200);
  Serial.println("Giro completado");
}


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

// - - SETUP - - //
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Motores
  detener();

  // Ultrasonicos
  pinMode(TRIG_PIN_R, OUTPUT); pinMode(ECHO_PIN_R, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_L, OUTPUT); pinMode(ECHO_PIN_L, INPUT);

  // BNO085
  selectChannel(BNO_C);
  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("No se detecta BNO085");
    while (1);
  }
  bno08x.enableReport(SH2_ROTATION_VECTOR);
  delay(200);

  yawInicial = obtenerYaw();

  // RGB
  selectChannel(RGB_C);
  if (!tcs.begin()) {
    Serial.println("Sensor TCS34725 encontrado.");
    while (1); 
  }

  // LCD
  selectChannel(LCD_C);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Wall E");

  // Servo
  garra.attach(GARRA_P);
  garra.write(135);

  // IR
  pinMode(IR_PIN, INPUT);
}

// - - LOOP - - //
void loop() {
  int velocidad = 70;
  int velocidadGiro = 150;

  long distF = obtenerDistancia(TRIG_PIN, ECHO_PIN);
  long distL = obtenerDistancia(TRIG_PIN_L, ECHO_PIN_L);
  long distR = obtenerDistancia(TRIG_PIN_R, ECHO_PIN_R);
  String color = Color();
  actualizarLCD(distF, color);

  int irValor = digitalRead(IR_PIN);

  if (!recorridoIniciado && color == "Verde") {
    recorridoIniciado = true;
    Serial.println("Inicio detectado (verde)");
    detener();
    delay(500);
    adelante(velocidad);
  }

  if (!recorridoIniciado) {
    detener();
    return;
  }

  if (irValor == LOW && !tienePelota) {
    detener();
    Serial.println("IR activado - cerrando garra");
    garra.write(115);
    delay(700);
    tienePelota = true;
    adelante(velocidad);
  }

  if (color == "Blanco") {
    puedeLeerBandera = true;
  }

  if (tienePelota && puedeLeerBandera) {
    if (color == "Azul") {
      Serial.println("Bandera Azul - Soltar pelota");
      detener();
      delay(2000);
      garra.write(135);
      atras(velocidad);
      delay(750);
      adelante(velocidad);
      delay(750);
      garra.write(115);
      delay(1000);
      adelante(velocidad);
      puedeLeerBandera = false;
    }
    else if (color == "Amarillo") {
      Serial.println("Bandera Amarilla - Patear pelota");
      detener();
      delay(1000);
      garra.write(135);
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

  if (color == "Rojo") {
    Serial.println("Fin detectado (rojo)");
    adelante(velocidad);
    delay(350);
    detener();
    recorridoIniciado = false;
    tienePelota = false;
  }

  // Evita obstáculos
  if ((distF < 7.5) && (distL < 12)) {
    girar(88, true, velocidadGiro);   // derecha
  }
  else if ((distF < 7.5) && (distR < 12)) {
    girar(86, false, velocidadGiro);  // izquierda
  }
  else {
    adelante(velocidad);
  }
}
