// Por Max
// Robot que sigue línea con el QTR-8A y motores del shield
// La idea: calibro cada sensor (min/max), normalizo a 0..1000 (0=negro, 1000=blanco),
// saco la “posición” de la línea y muevo motores con un control proporcional.
// También guardo la última acción para no perderme si la línea se va tantito.

#include <AFMotor.h>

// Pines del QTR-8A en el Mega (A8..A15)
const uint8_t QTR_PINS[] = {A8, A9, A10, A11, A12, A13, A14, A15};
const uint8_t N          = sizeof(QTR_PINS) / sizeof(QTR_PINS[0]);  // cuántos sensores tengo
const uint8_t LEDON_PIN  = 45; // este prende los emisores IR

// Motores (Adafruit Motor Shield v1)
AF_DCMotor MDD(4);  // derecha delantero
AF_DCMotor MDI(3);  // izquierda delantero
AF_DCMotor MTD(1);  // derecha trasero
AF_DCMotor MTI(2);  // izquierda trasero

// Calibración por sensor: aquí guardo el mínimo y máximo que vio cada uno.
uint16_t sMin[N], sMax[N];
const unsigned long CAL_MS = 3000; // me doy 3s para pasar calibrar

// Velocidades/control
const int BASE_SPEED = 70;   // velocidad base
const int TURN_GAIN  = 300;  // qué tan fuerte corrijo (proporcional)

// Histeresis para decidir si estoy centrado o ya me fui a un lado
const int CENTER_ENTER = 300; // para volver al centro
const int CENTER_EXIT  = 500; // para salir del centro a izquierda/derecha

// Giros cortos “especiales” (cuando cruzo del centro a un lado)
const int TURN_FWD  = 150;
const int TURN_BACK = -160;
const int TURN_MS   = 200;

enum Zone { Z_LEFT, Z_CENTER, Z_RIGHT };
Zone zone = Z_CENTER, lastZone = Z_CENTER;  // dónde creo que estoy y de dónde vengo

// Memoria de última instrucción para no paniquear si pierdo la línea
enum Action { ACT_FWD, ACT_LEFT, ACT_RIGHT };
Action lastAction = ACT_FWD;
unsigned long lastLineTime = 0;
const unsigned long LOST_MAX_MS = 1500;   // si pierdo la línea menos de 1.5s, sigo con lo último

// Umbrales rápidos para detectar “todos blancos” o “todos negros” y si hay línea
const int ALL_LOW_THR  = 100;   // negro (normalizado bajo)
const int ALL_HIGH_THR = 900;   // blanco  (normalizado alto)
const int ACTIVE_THR   = 200;   // digo “hay línea” si el valor pasa de aquí

// Lectura promediada por si hay ruido
int readAveraged(uint8_t pin, uint8_t samples = 6) {
  long acc = 0; for (uint8_t k=0;k<samples;k++) acc += analogRead(pin);
  return (int)(acc / samples);
}

// Limitar cualquier valor a -255 a 255 para el shield
int clamp255(int x){ if(x<-255)return -255; if(x>255)return 255; return x; }

// Mover un lado (par de motores) con mismo signo y magnitud
void driveSide(AF_DCMotor& a, AF_DCMotor& b, int spd){
  if (spd==0){ a.run(RELEASE); b.run(RELEASE); a.setSpeed(0); b.setSpeed(0); return; }
  bool fwd = (spd>=0);
  int s = clamp255(spd>=0? spd : -spd);
  a.run(fwd? FORWARD : BACKWARD);
  b.run(fwd? FORWARD : BACKWARD);
  a.setSpeed(s);
  b.setSpeed(s);
}

// Controlar izquierdo/derecho con valores independientes
void driveLR(int left, int right){
  driveSide(MDI, MTI, left);
  driveSide(MDD, MTD, right);
}

// Giros cortitos cuando cruzo del centro a un lado
void specialTurnRight(){ driveLR(TURN_FWD, TURN_BACK); delay(TURN_MS); }
void specialTurnLeft(){  driveLR(TURN_BACK, TURN_FWD); delay(TURN_MS); }

// Calibración: detecto min/max de cada sensor moviendo el arreglo sobre blanco y negro
void calibrateSensors(){
  for(int i=0;i<N;i++){ sMin[i]=1023; sMax[i]=0; }
  unsigned long t0=millis();
  while (millis()-t0 < CAL_MS){
    for(int i=0;i<N;i++){
      int v = readAveraged(QTR_PINS[i]);
      if(v<sMin[i]) sMin[i]=v;
      if(v>sMax[i]) sMax[i]=v;
    }
    delay(10);
  }
}

// Con histeresis: si estoy en centro, necesito salir más para declarar izquierda/derecha
Zone computeZone(int error){
  int aerr = error>=0? error : -error;
  if (zone==Z_CENTER){
    if (error >= CENTER_EXIT) return Z_RIGHT;
    if (error <= -CENTER_EXIT) return Z_LEFT;
    return Z_CENTER;
  } else if (zone==Z_LEFT){
    if (aerr <= CENTER_ENTER) return Z_CENTER;
    return Z_LEFT;
  } else {
    if (aerr <= CENTER_ENTER) return Z_CENTER;
    return Z_RIGHT;
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(LEDON_PIN, OUTPUT);
  digitalWrite(LEDON_PIN, HIGH); // prendo IR siempre

  driveLR(0,0); // arrancar quieto
  calibrateSensors(); // aquí aprende qué es blanco/negro por sensor
  zone = lastZone = Z_CENTER; // empiezo asumiendo centro
  lastAction = ACT_FWD; // y que voy derecho
  lastLineTime = millis(); // última vez que “vi” línea
}

void loop(){
  // Normalizo a 0..1000 (0=negro, 1000=blanco) usando lo que aprendí en calibración
  uint16_t norm[N];
  int maxNorm = 0; // el valor más alto que veo (para “hay línea”)
  int cntLow = 0, cntHigh = 0; // contadores para “todos blancos” / “todos negros”

  for(int i=0;i<N;i++){
    int v = analogRead(QTR_PINS[i]); // lectura de datos crudos 0..1023
    int span = sMax[i]-sMin[i]; if (span<1) span=1; // por si algún sensor quedó plano

    // Mapeo lineal: si v = sMax => negro (1000); si v = sMin => blanco (0)
    long nb = ((long)sMax[i]-v)*1000L/span;
    if(nb<0) nb=0; if(nb>1000) nb=1000;
    norm[i]=(uint16_t)nb;

    // Estadística rápida por frame
    if (norm[i] < ALL_LOW_THR)  cntLow++; // casi blanco
    if (norm[i] > ALL_HIGH_THR) cntHigh++; // casi negro
    if (norm[i] > maxNorm)      maxNorm = norm[i];
  }

  // Caso borde: si todo se ve igual (todo claro o todo oscuro), mejor avanzo recto
  if (cntLow == N || cntHigh == N) {
    driveLR(BASE_SPEED, BASE_SPEED);
    lastAction = ACT_FWD; // memorizo que voy derecho
    lastLineTime = millis(); // sí vi “algo coherente”
    lastZone = Z_CENTER; // me quedo en centro
    delay(20);
    return;
  }

  // ¿Hay línea razonable en lo que veo? Si no hay nada fuerte, uso memoria
  bool hasLine = (maxNorm > ACTIVE_THR);

  // Regla extra: si esquinas ven negro (0) y el centro está altísimo, forzar giro a la izquierda duro
  if (hasLine) {
    bool cornersLow = (norm[0] <= ALL_LOW_THR) && (norm[N-1] <= ALL_LOW_THR);
    int cAvg = (norm[(N/2)-1] + norm[N/2]) / 2;// promedio de los dos del centro
    if (cornersLow && cAvg >= 950) {
      driveLR(180, -180);// giro sobre eje a la izquierda
      lastAction   = ACT_RIGHT;// guardo última “tendencia” (tu lógica)
      lastZone     = Z_RIGHT; // y zona
      lastLineTime = millis();
      delay(60)
      return;
    }
  }

  // Si perdí la línea: mantengo la última intención por un rato, luego busco girando
  if (!hasLine) {
    unsigned long dt = millis() - lastLineTime;
    if (dt <= LOST_MAX_MS) {
      // sigo lo último que estaba haciendo
      if (lastAction == ACT_FWD) {
        driveLR(BASE_SPEED, BASE_SPEED);
      } else if (lastAction == ACT_LEFT) {
        driveLR(BASE_SPEED - 30, BASE_SPEED + 30); // sesgo a la izq
      } else {
        driveLR(BASE_SPEED + 30, BASE_SPEED - 30); // sesgo a la der
      }
    } else {
      // ya pasó mucho sin ver línea: giro en sitio para encontrarla
      driveLR(-150, 150);
    }
    delay(20);
    return;
  }

  // Hay línea: calculo posición 0..7000 (centro=3500) y error -3500..+3500
  long num=0, den=0;
  for(int i=0;i<N;i++){
    num += (long)norm[i] * (i*1000); // peso por índice (centroide 1D)
    den += norm[i];
  }
  int pos7000 = den ? (int)(num/den) : 3500; // si algo raro, me quedo en centro
  int error   = pos7000 - 3500; // negativo=me fui a la izq, positivo=der

  // Actualizo zona con histeresis y aplico un “snap” corto si cambié de centro a lado
  zone = computeZone(error);
  if (lastZone==Z_CENTER && zone==Z_RIGHT){
    specialTurnRight();
  } else if (lastZone==Z_CENTER && zone==Z_LEFT){
    specialTurnLeft();
  }

  // Control proporcional sencillo: más error, más diferencia entre llantas
  long D = (long)error * TURN_GAIN / 3500;
  int left  = clamp255(BASE_SPEED + (int)D);
  int right = clamp255(BASE_SPEED - (int)D);
  driveLR(left, right);

  // Memorizo hacia dónde iba para el modo “perdido”
  if (error > CENTER_ENTER)       lastAction = ACT_RIGHT;
  else if (error < -CENTER_ENTER) lastAction = ACT_LEFT;
  else                            lastAction = ACT_FWD;

  lastLineTime = millis();
  lastZone = zone;
  delay(20);  // no tan alto para no laggear, no tan bajo para no vibrar
}