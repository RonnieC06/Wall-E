//Por Max
// Lee los sensores QTR-8A de forma analógica y muestra valores entre 0 y 1000
// Básicamente calibra primero para saber qué es “negro” y qué es “blanco”,
// Como el rango de valores de nuestro pololu era entre 900 y 1000, había que normalizar
//Con la normalización imprime los valores entre 0 (negro) y 1000 (blanco)

const uint8_t QTR_PINS[] = {A8, A9, A10, A11, A12, A13, A14, A15};
const uint8_t N = sizeof(QTR_PINS) / sizeof(QTR_PINS[0]); // Cuenta cuántos sensores hay
const uint8_t LEDON_PIN = 45;  // Pin que enciende los LEDs del sensor

uint16_t sMin[N], sMax[N];// Guarda los valores mínimos y máximos detectados en calibración
const unsigned long CAL_MS = 3000;// Tiempo que dura la calibración

void setup() {
  Serial.begin(115200);
  pinMode(LEDON_PIN, OUTPUT);
  digitalWrite(LEDON_PIN, HIGH);  // Activa los emisores IR del Pololu

  Serial.println("=== Calibrando QTR-8A ===");
  Serial.println("Mueve el sensor sobre blanco y negro durante 3s...");

  calibrateSensors();
  // Se hace la calibración guardando min y max de cada sensor
  // ya que todos tienen rangos diferentes y unos el valor minimo superaba el máximo de otros
  Serial.println("=== Calibración completa ===");
  Serial.println("Leyendo valores normalizados (0=blanco, 1000=negro)");
  Serial.println("-----------------------------------------------");
}

void loop() {
  uint16_t norm[N];  // Aquí se guardan los valores ya normalizados

  for (int i = 0; i < N; i++) {
    int v = analogRead(QTR_PINS[i]);  // Lee el valor crudo (0 a 1023)
    int span = sMax[i] - sMin[i];// Diferencia entre el valor más claro y el más oscuro
    if (span < 1) span = 1;// Evita dividir entre cero

    // Convierte el valor crudo a la escala de 0 a 1000
    long nb = ((long)sMax[i] - v) * 1000L / span;
    if (nb < 0) nb = 0;
    if (nb > 1000) nb = 1000;
    norm[i] = (uint16_t)nb;
  }
  // Imprime los valores de cada sensor
  for (int i = 0; i < N; i++) {
    Serial.print(norm[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(50);
}

void calibrateSensors() {
  // Inicializa los mínimos y máximos en los extremos
  for (int i = 0; i < N; i++) {
    sMin[i] = 1023;
    sMax[i] = 0;
  }

  unsigned long t0 = millis(); // Durante 3 segundos va leyendo cada sensor y actualizando sus min y max
  while (millis() - t0 < CAL_MS) {
    for (int i = 0; i < N; i++) {
      int v = analogRead(QTR_PINS[i]);
      if (v < sMin[i]) sMin[i] = v;
      if (v > sMax[i]) sMax[i] = v;
    }
    delay(10);
  }

  // Al final imprime los valores mínimos y máximos detectados por cada sensor
  Serial.println("Mínimos y máximos detectados por sensor:");
  for (int i = 0; i < N; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(": min="); Serial.print(sMin[i]);
    Serial.print(", max="); Serial.println(sMax[i]);
  }
  Serial.println();
}