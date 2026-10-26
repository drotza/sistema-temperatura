// Parte 2: control PID de temperatura (lazo cerrado) con referencia ajustable.
// Todo en unidades crudas: medicion y referencia en cuentas del ADC (0..1023),
// salida en cuentas de PWM (0..255). No se convierte a °C.
//
// Termistor NTC 10k B3950. Cartucho 12V 40W alimentado con pila de 9V.
//
// Conexiones:
//   D3 -> 1k -> TIP121 base (pata 1)
//   TIP121 colector (pata 2) -> calentador -> +9V (pila)
//   TIP121 emisor   (pata 3) -> GND (comun con la pila -)
//   +5V -> termistor -> A0 -> R2 (1k) -> GND   (~120 cuentas a ~30 °C)
//   Con el termistor arriba del divisor, mas temperatura = mas cuentas de ADC.
//
// Protocolo serial (115200, una orden por linea):
//   L               empieza a enviar datos
//   R<adc>          referencia en cuentas de ADC   ej. R180
//   K<kp>,<ki>,<kd> ganancias del PID (PWM por cuenta de ADC)  ej. K9.4,0.157,0
//   A               modo automatico (lazo cerrado, PID)
//   M<pwm>          modo manual (lazo abierto), 0..255  ej. M100
//   E<0|1>          habilita el calentador. La salida real es PID AND habilitado:
//                   con E0 el pin queda en 0 sin importar lo que pida el control
//   X               deshabilita, apaga y deja de enviar
// Cada TS_MS envia: t_s,ref_adc,adc,pwm,modo,hab
//   pwm = lo que realmente sale por D3 (0 si esta deshabilitado)
//   modo 1 = PID, 0 = manual;  hab 1 = habilitado, 0 = apagado

const uint8_t HEATER_PIN = 3;      // D3: PWM por hardware (analogWrite 0..255)
const uint8_t TEMP_PIN   = A0;
const unsigned long TS_MS = 500;   // periodo de muestreo / control

float Kp = 9.4, Ki = 0.157, Kd = 0.0;  // PI por SIMC (tc=10 s); PWM/cuenta, PWM/(cuenta*s)
float ref = 180.0;                     // ~43 °C con NTC 10k B3950 y R2 = 1k
bool  modoAuto = true;
bool  habilitado = false;              // arranca apagado: hay que habilitar desde la GUI
float uMan = 0.0;

float u    = 0.0;   // salida que pide el control 0..255
float I    = 0.0;   // termino integral
float dF   = 0.0;   // derivada (filtrada) de la medicion
float yAnt = NAN;
float adc  = NAN;

bool registrando = false;
unsigned long tInicio, tSig;

char buf[40];
uint8_t nbuf = 0;


float leerAdc() {
  long s = 0;
  for (int i = 0; i < 16; i++) s += analogRead(TEMP_PIN);   // promedio para bajar ruido
  return s / 16.0;
}


// PID paralelo: u = Kp*e + Ki*integral(e) + Kd*d(-y)/dt
//  - derivada sobre la medicion (sin "patada" al cambiar la referencia)
//  - anti-windup: no se integra si la salida esta saturada y el error la empuja mas,
//    ni mientras el calentador este deshabilitado
void controlar(float y) {
  const float Ts = TS_MS / 1000.0;
  float e = ref - y;
  if (!isnan(yAnt)) dF += 0.3 * (-(y - yAnt) / Ts - dF);
  yAnt = y;
  if (!modoAuto) { u = uMan; return; }

  float v = Kp * e + I + Kd * dF;
  u = constrain(v, 0.0, 255.0);
  if (habilitado && !((v >= 255.0 && e > 0) || (v <= 0.0 && e < 0))) I += Ki * Ts * e;
  I = constrain(I, -255.0, 255.0);
}

// Transferencia sin salto: ajusta la integral para que la salida siga igual
// al pasar de manual a automatico o al cambiar ganancias.
void sinSalto() {
  if (isnan(adc)) return;
  I = constrain(u - Kp * (ref - adc) - Kd * dF, -255.0, 255.0);
}


uint8_t salida() {                 // lo que realmente sale: control AND habilitado
  return habilitado ? (uint8_t)(u + 0.5) : 0;
}

void aplicarSalida() {
  analogWrite(HEATER_PIN, salida());
}


void comando(char *c) {
  switch (c[0]) {
    case 'L':
      registrando = true;
      tInicio = millis();
      Serial.println(F("t_s,ref_adc,adc,pwm,modo,hab"));
      break;
    case 'R':
      ref = constrain(atof(c + 1), 0.0, 1023.0);
      Serial.print(F("# Referencia = ")); Serial.println(ref, 0);
      break;
    case 'K': {
      char *p = c + 1;
      Kp = atof(p);
      if ((p = strchr(p, ','))) { Ki = atof(++p);
        if ((p = strchr(p, ','))) Kd = atof(++p); }
      sinSalto();
      Serial.print(F("# Kp=")); Serial.print(Kp, 3);
      Serial.print(F(" Ki=")); Serial.print(Ki, 4);
      Serial.print(F(" Kd=")); Serial.println(Kd, 3);
      break;
    }
    case 'A':
      if (!modoAuto) { modoAuto = true; sinSalto(); }
      Serial.println(F("# Modo PID (lazo cerrado)"));
      break;
    case 'M':
      modoAuto = false;
      uMan = constrain(atof(c + 1), 0.0, 255.0);
      u = uMan;
      aplicarSalida();
      Serial.print(F("# Modo manual, PWM = ")); Serial.println(uMan, 0);
      break;
    case 'E':
      habilitado = atoi(c + 1) != 0;
      aplicarSalida();
      Serial.println(habilitado ? F("# Calentador HABILITADO") : F("# Calentador APAGADO"));
      break;
    case 'X':
      registrando = false;
      habilitado = false;
      modoAuto = false;
      uMan = u = 0.0;
      aplicarSalida();
      Serial.println(F("# Detenido"));
      break;
    default:
      Serial.print(F("# Orden desconocida: ")); Serial.println(c);
  }
}


void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  Serial.begin(115200);
  tSig = millis();
  Serial.println(F("# Listo"));
}


void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (nbuf) { buf[nbuf] = 0; comando(buf); nbuf = 0; }
    } else if (nbuf < sizeof(buf) - 1) {
      buf[nbuf++] = ch;
    }
  }

  unsigned long ahora = millis();
  if ((long)(ahora - tSig) >= 0) {
    tSig += TS_MS;
    adc = leerAdc();
    controlar(adc);
    aplicarSalida();

    if (registrando) {
      Serial.print((ahora - tInicio) / 1000.0, 1); Serial.print(',');
      Serial.print(ref, 0);                        Serial.print(',');
      Serial.print(adc, 2);                        Serial.print(',');
      Serial.print(salida());                      Serial.print(',');
      Serial.print(modoAuto ? 1 : 0);              Serial.print(',');
      Serial.println(habilitado ? 1 : 0);
    }
  }
}
