// - - LIBRERIAS Y DECLARACIONES NECESARIAS PARA FUNCIONAR - - //

// Librerías
#include <AFMotor.h> // Motores
#include <Wire.h> // Multiplexor
#include <Adafruit_TCS34725.h> // RGB
#include <Adafruit_BNO08x.h> // BNO
#include <LiquidCrystal_I2C.h> // LCD

// Motores
AF_DCMotor MDD(3); // Motor delantero derecho
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

// Multiplexor y Arduino
#define MPX 0x70
#define LCD_C 1
#define RGB_C 4
#define BNO_C 5
#define LCD_ADDR 0x27

// Objetos
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// Variables BNO
float yawInicial = 0;
float yawActual = 0;

// Variables generales
bool mazeIniciado = false;
static bool regreso = false;
bool subiendo = false;
bool giroDerechaSiguiente = false;  // false = izquierda, true = derecha
bool puedeGirarAmbosLados = true;
bool verdeDetectado = false;
int contadorGiros = 0;
unsigned long ultimoGiroAmbosLados = 0; // Tiempo del último giro
const unsigned long intervaloGiroAmbosLados = 8000; // Ajustar según lo requerido

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

// - - FUNCIONES DE SENSORES - - //
// Ultrasonicos
long obtenerDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH, 10000);
  return duracion * 0.034 / 2;
}

// RGB
String Color() {
  uint16_t r, g, b, c;
  selectChannel(RGB_C);
  tcs.getRawData(&r, &g, &b, &c);

  if ((r >= 11 && r <= 32) && (g >= 21 && g <= 42) && (b >= 6 && b <= 25)) return "Verde";
  else if ((r >= 45 && r <= 85) && (g >= 7 && g <= 24) && (b >= 5 && b <= 24)) return "Rojo";
  else if ((r >= 6 && r <= 22) && (g >= 14 && g <= 30) && (b >= 18 && b <= 34)) return "Azul";
  else if ((r >= 100 && r <= 150) && (g >= 80 && g <= 120) && (b >= 15 && b <= 55)) return "Amarillo";
  else if ((r >= 80 && r <= 120) && (g >= 29 && g <= 55) && (b >= 25 && b <= 50)) return "Rosa";
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

  while (true) {
    float yaw = obtenerYaw();
    float error = fabs(yaw - objetivo);

    if (error > 180) error = 360 - error;

    if (error <= 5) break;

    if (derecha) giro_derecha(velocidadGiro);
    else giro_izquierda(velocidadGiro);
  }

  detener();
  delay(200);
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

  // Motores detenidos
  detener();

  // Ultrasonidos
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
    Serial.println("Error: no se encontró el sensor RGB."); 
    while (1); 
  }

  // LCD
  selectChannel(LCD_C);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Robot Maze Ready");
}

// - - LOOP - - //
void loop() {
  int velocidad = 80;
  int velocidadGiro = 140;

  long distF = obtenerDistancia(TRIG_PIN, ECHO_PIN);
  long distL = obtenerDistancia(TRIG_PIN_L, ECHO_PIN_L);
  long distR = obtenerDistancia(TRIG_PIN_R, ECHO_PIN_R);
  String color = Color();
  actualizarLCD(distF, color);

  // Inicio del Maze al detectar color verde
  if (!mazeIniciado && color == "Verde") {
    mazeIniciado = true;
    adelante(velocidad);
    delay(400);
  }

  if (!mazeIniciado && !regreso) {
    detener();
    return;
  }

  // Retrocede si detecta negro
  if (color == "Negro") {
    detener();
    atras(100);
    delay(500);
    detener();
    girar(89, false, velocidadGiro); // Gira izquierda al encontrar negro
    return;
  }

  // Si detecta rojo termina maze y continua sube y baja
  if (color == "Rojo" && !regreso && !subiendo) {
    adelante(velocidad);
    delay(350);
    detener();
    delay(800);

    subiendo = true;  // Ignorar sensores mientras sube

    // Activa modo regreso después de subir
    regreso = true;
    mazeIniciado = false;
    return;
  }

  if (regreso) {

  // Si aún no ha detectado el verde, avanzar poco a poco hasta encontrarlo
    if (!verdeDetectado) {
      while (true) {
        adelante(150);
        delay(250);   // Ajustar velocidad y tiempo para tener potencia necesaria
        detener();
        delay(150);

        String colorActual = Color();

        if (colorActual == "Verde") {
          verdeDetectado = true;
          detener();
          delay(300);
          girar(175, false, 140);
          break;
        }
      }
    }

    // Ya detectó el verde, ahora retrocede hasta encontrar rojo
    if (verdeDetectado) {
      while (true) {
        adelante(120);
        delay(200);
        detener();
        delay(100);

        String nuevoColor = Color();

        if (nuevoColor == "Rojo") {
          detener();
          regreso = false;
          mazeIniciado = false;
          subiendo = false;
          verdeDetectado = false; 
          return;
        }
      }
    }

  return; // sigue en modo regreso
  }





  // Lógica de evasión de obstáculos
  if (!subiendo && !regreso) {
    if ((distF < 9) && (distR < 12)) {
      girar(89, false, velocidadGiro);
      
      contadorGiros++;

      if (contadorGiros > 7) {
        giroDerechaSiguiente = !giroDerechaSiguiente;
        contadorGiros = 0;
      }
    }
    else if ((distF < 9) && (distL < 12)) {
      girar(89, true, velocidadGiro);
      
      contadorGiros++;

      if (contadorGiros > 7) {
        giroDerechaSiguiente = !giroDerechaSiguiente;
        contadorGiros = 0;
      }
    }
    else if ((distL > 13) && (distR > 13)) {
      unsigned long tiempoActual = millis();

  // Solo permitir giro si ha pasado suficiente tiempo para evitar loops
      if (tiempoActual - ultimoGiroAmbosLados > intervaloGiroAmbosLados) {
        ultimoGiroAmbosLados = tiempoActual;

        adelante(velocidad);
        delay(300);

        if (giroDerechaSiguiente) {
          girar(88, true, velocidadGiro);
        } else {
          girar(90, false, velocidadGiro);
        }

        contadorGiros++;
        if (contadorGiros > 7) {
          giroDerechaSiguiente = !giroDerechaSiguiente;
          contadorGiros = 0;
        }
      }
      else {
        // Si aún no pasa el tiempo, solo avanza un poco
        adelante(velocidad);
      }
    }
    else if ((distF < 10) && (distL < 10) && (distR < 10)) {
      atras(velocidad);
      delay(1000);
      if (giroDerechaSiguiente) {
        girar(89, false, velocidadGiro);
      } else {
        girar(89, true, velocidadGiro);
      }

      contadorGiros++;

      if (contadorGiros > 7) {
        giroDerechaSiguiente = !giroDerechaSiguiente;
        contadorGiros = 0;
      }
    }
    else if (distF < 8) {
      if (giroDerechaSiguiente) {
        girar(90, true, velocidadGiro);
      } else {
        girar(90, false, velocidadGiro);
      }

      contadorGiros++;

      if (contadorGiros > 7) {
        giroDerechaSiguiente = !giroDerechaSiguiente;
        contadorGiros = 0;
      }
    } 
    else {
      adelante(velocidad);
    }
  }
}
