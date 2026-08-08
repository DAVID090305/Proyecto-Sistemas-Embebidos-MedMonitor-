#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <LittleFS.h>
#include "Audio.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// LED_VERDE y LED_AMARILLO estan en GPIO4/5 (se movieron de GPIO25/26 para
// liberar esos pines, que ahora usamos para I2S / audio).
#define LED_VERDE 4
#define LED_AMARILLO 5
#define LED_ROJO 27

#define BTN_SIGUIENTE 18
#define BTN_REINICIAR 19

// --- Audio (MAX98357A por I2S) ---
#define I2S_BCLK 26
#define I2S_LRC 25
#define I2S_DOUT 14

// Potenciometro de volumen (cursor a este pin, extremos a 3.3V y GND)
#define VOLUME_POT_PIN 34

#define EEPROM_SIZE 200
#define SSID1_ADDR 0
#define PASS1_ADDR 50
#define SSID2_ADDR 100
#define PASS2_ADDR 150

#define MAX_REGISTROS 10

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;

// Bus I2C #2, exclusivo para el MAX30102
TwoWire I2C_PULSO = TwoWire(1);
#define SDA2_PIN 32
#define SCL2_PIN 33

WebServer server(80);
Audio audio;

const char* ap_ssid = "MedMonitor_AP";
const char* ap_password = "12345678";

const byte RATE_SIZE = 10;
byte rates[RATE_SIZE];
byte rateSpot = 0;
byte bpmContador = 0;

long lastBeat = 0;
float bpm = 0;
int bpmProm = 0;
long irValue = 0;

float temp = 0;
float tempAmb = 0;

// --- Oximetro de pulso (SpO2) ---
#define SPO2_BUFFER_SIZE 100
uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];
int spo2BufferIndex = 0;

int32_t spo2Calculado = 0;
int8_t spo2Valido = 0;
int32_t hrCalculado = 0;
int8_t hrValidoAlgoritmo = 0;

int spo2Actual = 0;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 500;

bool heartBig = false;

// --- Carrusel de slides del OLED (BPM / SpO2 / Temp), controlado por botones ---
int slideActual = 0;
const int TOTAL_SLIDES = 3;

// Antirrebote (debounce) para los 2 pulsadores
const unsigned long DEBOUNCE_DELAY = 50; // ms

bool estadoSiguiente = HIGH;
bool ultimaLecturaSiguiente = HIGH;
unsigned long ultimoDebounceSiguiente = 0;

bool estadoReiniciar = HIGH;
bool ultimaLecturaReiniciar = HIGH;
unsigned long ultimoDebounceReiniciar = 0;

// Captura automatica unificada
bool dedoPuesto = false;
unsigned long dedoPuestoDesde = 0; // millis() de cuando empezo el contacto actual
int ultimoBpmValido = 0;
int bpmFinalAuto = 0;
bool bpmCapturado = false;

int ultimoSpo2Valido = 0;
int spo2FinalAuto = 0;
bool spo2Capturado = false;

float tempSumAuto = 0;
int tempCountAuto = 0;
float tempFinalAuto = 0;
bool tempCapturada = false;
bool tempPresente = false; // "objeto en rango valido" equivalente al dedoPuesto
unsigned long tempPresenteDesde = 0; // millis() de cuando empezo el contacto actual

// --- Maquina de estados del flujo guiado: BPM+SpO2 -> Temperatura -> Final ---
enum EtapaMedicion { ETAPA_BPM_SPO2, ETAPA_TEMP, ETAPA_FINAL };
EtapaMedicion etapaActual = ETAPA_BPM_SPO2;

struct Registro {
  String nombre;
  int edad;
  int bpm;
  int spo2;
  float temp;
  String estado;   // gravedad combinada (Normal/Advertencia/Alerta) -> color
  String chequeo;  // resumen clinico legible para mostrar en el historial
};

Registro historial[MAX_REGISTROS];
int totalRegistros = 0;

// --- Cola de audio ---
// Permite encolar varias frases para que se reproduzcan una despues de la
// otra sin cortarse (la libreria interrumpe el audio actual si le pedis
// reproducir uno nuevo antes de que termine, por eso encolamos en vez de
// llamar a audio.connecttoFS() directo cuando puede haber mas de una frase
// seguida).
#define AUDIO_QUEUE_SIZE 4
String colaAudio[AUDIO_QUEUE_SIZE];
int colaAudioLen = 0;

void encolarAudio(const char* ruta) {
  if (colaAudioLen < AUDIO_QUEUE_SIZE) {
    colaAudio[colaAudioLen] = String(ruta);
    colaAudioLen++;
  }
}

void limpiarColaAudio() {
  colaAudioLen = 0;
}

void procesarColaAudio() {
  if (!audio.isRunning() && colaAudioLen > 0) {
    String siguiente = colaAudio[0];
    for (int i = 1; i < colaAudioLen; i++) colaAudio[i - 1] = colaAudio[i];
    colaAudioLen--;
    audio.connecttoFS(LittleFS, siguiente.c_str());
  }
}

// --- Volumen por potenciometro (GPIO34) ---
int volumenActual = -1; // -1 fuerza que se aplique la primera lectura
unsigned long ultimaLecturaVolumen = 0;
const unsigned long INTERVALO_LECTURA_VOLUMEN = 200; // ms

void actualizarVolumen() {
  if (millis() - ultimaLecturaVolumen < INTERVALO_LECTURA_VOLUMEN) return;
  ultimaLecturaVolumen = millis();

  int lectura = analogRead(VOLUME_POT_PIN); // 0-4095
  int nuevoVolumen = map(lectura, 0, 4095, 0, 21);
  nuevoVolumen = constrain(nuevoVolumen, 0, 21);

  if (nuevoVolumen != volumenActual) {
    volumenActual = nuevoVolumen;
    audio.setVolume(volumenActual);
  }
}

void escribirStringEEPROM(int direccion, String texto) {
  for (int i = 0; i < texto.length(); i++) EEPROM.write(direccion + i, texto[i]);
  EEPROM.write(direccion + texto.length(), '\0');
  EEPROM.commit();
}

String leerStringEEPROM(int direccion) {
  String texto = "";
  char c;
  while ((c = EEPROM.read(direccion++)) != '\0') texto += c;
  return texto;
}

void apagarSemaforo() {
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_AMARILLO, LOW);
  digitalWrite(LED_ROJO, LOW);
}

String obtenerEstadoBPM(int bpm) {
  if (bpm == 0) return "Midiendo";
  if (bpm >= 60 && bpm <= 100) return "Normal";
  if ((bpm >= 40 && bpm < 60) || (bpm > 100 && bpm <= 120)) return "Advertencia";
  return "Alerta";
}

String obtenerEstadoSpO2(int spo2) {
  if (spo2 == 0) return "Midiendo";
  if (spo2 >= 95) return "Normal";
  if (spo2 >= 90) return "Advertencia";
  return "Alerta";
}

String obtenerEstadoTemp(float t) {
  if (t == 0.0) return "Midiendo";
  if (t >= 36.0 && t <= 37.5) return "Normal";
  if ((t >= 35.0 && t < 36.0) || (t > 37.5 && t <= 38.5)) return "Advertencia";
  return "Alerta";
}

// --- Etiquetas clinicas especificas por parametro (solo para mostrar en pantalla) ---
// Nota: la gravedad interna para el semaforo sigue usando obtenerEstadoBPM/SpO2/Temp
// (Normal/Advertencia/Alerta) sin cambios; esto es unicamente el texto visible.

String etiquetaBPM(int bpm) {
  if (bpm == 0) return "Sin datos";
  if (bpm < 40) return "Bradicardia severa";
  if (bpm < 60) return "Bradicardia";
  if (bpm <= 100) return "Ritmo estable";
  if (bpm <= 120) return "Taquicardia";
  return "Taquicardia severa";
}

String etiquetaSpO2(int spo2) {
  if (spo2 == 0) return "Sin datos";
  if (spo2 < 90) return "Hipoxia";
  if (spo2 < 95) return "Hipoxia leve";
  return "Saturacion optima";
}

String etiquetaTemp(float t) {
  if (t == 0.0) return "Sin datos";
  if (t < 35.0) return "Hipotermia";
  if (t < 36.0) return "Hipotermia leve";
  if (t <= 37.5) return "Temperatura estable";
  if (t <= 38.5) return "Febricula";
  return "Fiebre";
}

// Convierte un estado individual en puntos de gravedad para el sistema de puntaje:
// Normal = 0, Advertencia = 1, Alerta = 2
int puntosDeEstado(String estado) {
  if (estado == "Alerta") return 2;
  if (estado == "Advertencia") return 1;
  return 0; // Normal
}

// Veredicto final por sistema de puntos (similar a un Early Warning Score clinico):
// cada signo aporta puntos segun su gravedad, se suman, y el semaforo depende
// del puntaje total. Asi, una sola anomalia leve no dispara alerta roja completa,
// pero varias anomalias juntas si escalan el estado.
//
// Excepcion de seguridad: una SpO2 critica (<90%) siempre fuerza Alerta de
// inmediato, sin importar el resto, porque una hipoxia real es peligrosa por
// si sola y no debe "diluirse" en el puntaje.
String obtenerEstado(int bpm, int spo2, float t) {
  String estadoBpm = obtenerEstadoBPM(bpm);
  String estadoSpo2 = obtenerEstadoSpO2(spo2);
  String estadoTemp = obtenerEstadoTemp(t);

  if (estadoBpm == "Midiendo" || estadoSpo2 == "Midiendo" || estadoTemp == "Midiendo") {
    return "Midiendo";
  }

  if (estadoSpo2 == "Alerta") return "Alerta";

  int puntos = puntosDeEstado(estadoBpm) + puntosDeEstado(estadoSpo2) + puntosDeEstado(estadoTemp);

  if (puntos == 0) return "Normal";
  if (puntos <= 2) return "Advertencia";
  return "Alerta";
}

// Arma un resumen clinico legible para el historial del dashboard.
String obtenerResumenClinico(int bpm, int spo2, float t) {
  String estadoBpm = obtenerEstadoBPM(bpm);
  String estadoSpo2 = obtenerEstadoSpO2(spo2);
  String estadoTemp = obtenerEstadoTemp(t);

  if (estadoBpm == "Midiendo" || estadoSpo2 == "Midiendo" || estadoTemp == "Midiendo") {
    return "Medicion incompleta";
  }

  String resumen = "";
  if (estadoBpm != "Normal") resumen += etiquetaBPM(bpm);
  if (estadoSpo2 != "Normal") {
    if (resumen != "") resumen += "; ";
    resumen += etiquetaSpO2(spo2);
  }
  if (estadoTemp != "Normal") {
    if (resumen != "") resumen += "; ";
    resumen += etiquetaTemp(t);
  }

  if (resumen == "") return "Paciente estable";
  return resumen;
}

void actualizarSemaforo(String estado) {
  apagarSemaforo();
  if (estado == "Normal") digitalWrite(LED_VERDE, HIGH);
  else if (estado == "Advertencia") digitalWrite(LED_AMARILLO, HIGH);
  else if (estado == "Alerta") digitalWrite(LED_ROJO, HIGH);
}

void guardarRegistro(String nombre, int edad) {
  if (nombre == "") nombre = "Paciente";

  Registro nuevo;
  nuevo.nombre = nombre;
  nuevo.edad = edad;
  nuevo.bpm = bpmCapturado ? bpmFinalAuto : bpmProm;
  nuevo.spo2 = spo2Capturado ? spo2FinalAuto : spo2Actual;
  nuevo.temp = tempCapturada ? tempFinalAuto : temp;

  nuevo.estado = obtenerEstado(nuevo.bpm, nuevo.spo2, nuevo.temp);
  nuevo.chequeo = obtenerResumenClinico(nuevo.bpm, nuevo.spo2, nuevo.temp);

  if (totalRegistros < MAX_REGISTROS) {
    historial[totalRegistros] = nuevo;
    totalRegistros++;
  } else {
    for (int i = 0; i < MAX_REGISTROS - 1; i++) historial[i] = historial[i + 1];
    historial[MAX_REGISTROS - 1] = nuevo;
  }

  tempCapturada = false;
  bpmCapturado = false;
  spo2Capturado = false;
  tempFinalAuto = 0;
  bpmFinalAuto = 0;
  spo2FinalAuto = 0;

  etapaActual = ETAPA_BPM_SPO2;
  slideActual = 0;
  dedoPuesto = false;
  ultimoBpmValido = 0;
  ultimoSpo2Valido = 0;
  tempPresente = false;
  tempSumAuto = 0;
  tempCountAuto = 0;

  apagarSemaforo();

  // Nota: la despedida (audio9) ya se reprodujo automaticamente al terminar
  // de medir (justo despues del resultado). Guardar en el dashboard es una
  // accion silenciosa que no debe interrumpir el audio que este sonando.
}

void dibujarCorazon(int x, int y, bool grande) {
  if (grande) {
    display.fillCircle(x + 8, y + 8, 7, SH110X_WHITE);
    display.fillCircle(x + 22, y + 8, 7, SH110X_WHITE);
    display.fillTriangle(x + 1, y + 11, x + 29, y + 11, x + 15, y + 30, SH110X_WHITE);
  } else {
    display.drawCircle(x + 8, y + 8, 6, SH110X_WHITE);
    display.drawCircle(x + 20, y + 8, 6, SH110X_WHITE);
    display.drawTriangle(x + 2, y + 11, x + 26, y + 11, x + 14, y + 27, SH110X_WHITE);
  }
}

// Devuelve 0 a 3 puntos suspensivos que van creciendo con el tiempo.
String puntosAnimados() {
  int n = (millis() / 400) % 4;
  String p = "";
  for (int i = 0; i < n; i++) p += ".";
  return p;
}

// Dibuja un punto que crece y encoge en bucle (indicador de "grabando").
void dibujarPulso(int x, int y) {
  int fase = (millis() / 150) % 6;
  int radio = (fase < 3) ? (fase + 1) : (6 - fase);
  display.fillCircle(x, y, radio, SH110X_WHITE);
}

void mostrarSlideBPM() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("FRECUENCIA CARDIACA");

  dibujarCorazon(2, 16, heartBig);

  display.setTextSize(4);
  display.setCursor(46, 14);
  display.print(bpmFinalAuto);

  display.setTextSize(1);
  display.setCursor(0, 54);
  display.print(etiquetaBPM(bpmFinalAuto));
}

void mostrarSlideSpO2() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("OXIGENACION (SpO2)");

  display.setTextSize(4);
  display.setCursor(20, 18);
  display.print(spo2FinalAuto);
  display.setTextSize(2);
  display.print("%");

  display.setTextSize(1);
  display.setCursor(0, 54);
  display.print(etiquetaSpO2(spo2FinalAuto));
}

void mostrarSlideTemp() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("TEMPERATURA CORPORAL");

  display.setTextSize(4);
  display.setCursor(6, 18);
  display.print(tempFinalAuto, 1);

  display.setTextSize(2);
  display.print("C");

  display.setTextSize(1);
  display.setCursor(0, 54);
  display.print(etiquetaTemp(tempFinalAuto));
}

void mostrarEtapaBPMSpO2() {
  dibujarCorazon(2, 18, heartBig);

  display.setTextSize(1);
  display.setCursor(40, 14);
  display.print("BPM");
  display.setTextSize(2);
  display.setCursor(40, 26);
  if (bpmProm > 0) display.print(bpmProm);
  else display.print("--");

  display.setTextSize(1);
  display.setCursor(88, 14);
  display.print("SpO2");
  display.setTextSize(2);
  display.setCursor(88, 26);
  if (spo2Actual > 0) display.print(spo2Actual);
  else display.print("--");

  display.setTextSize(1);
  display.setCursor(0, 56);
  if (bpmCapturado || spo2Capturado) {
    display.print("Datos guardados");
    display.fillCircle(122, 5, 3, SH110X_WHITE);
  } else {
    display.print("Midiendo");
    display.print(puntosAnimados());
    dibujarPulso(122, 5);
  }
}

void mostrarEtapaTemp() {
  display.setTextSize(4);
  display.setCursor(10, 22);
  display.print(temp, 1);
  display.setTextSize(2);
  display.print("C");

  display.setTextSize(1);
  display.setCursor(0, 56);
  if (tempCapturada) {
    display.print("Datos guardados");
    display.fillCircle(122, 5, 3, SH110X_WHITE);
  } else {
    display.print("Midiendo");
    display.print(puntosAnimados());
    dibujarPulso(122, 5);
  }
}

void actualizarOLED(); // forward declaration, definida mas abajo

void actualizarSemaforoParaSlideActual() {
  switch (slideActual) {
    case 0: actualizarSemaforo(obtenerEstadoBPM(bpmFinalAuto)); break;
    case 1: actualizarSemaforo(obtenerEstadoSpO2(spo2FinalAuto)); break;
    case 2: actualizarSemaforo(obtenerEstadoTemp(tempFinalAuto)); break;
  }
}

void avanzarEtapa() {
  if (etapaActual == ETAPA_BPM_SPO2) {
    bool yaEstabaCapturado = bpmCapturado || spo2Capturado;

    // Resguardo: si el dedo sigue puesto y aun no se guardo, lo guardamos igual
    if (!bpmCapturado && bpmProm > 0) { bpmFinalAuto = bpmProm; bpmCapturado = true; }
    if (!spo2Capturado && spo2Actual > 0) { spo2FinalAuto = spo2Actual; spo2Capturado = true; }

    // Si por algun motivo no sonaron todavia (ej: presiono el boton sin
    // retirar el dedo primero), las encolamos aca como resguardo.
    if (!yaEstabaCapturado && (bpmCapturado || spo2Capturado)) {
      encolarAudio("/a3_indic2.mp3");
      encolarAudio("/a4_medirtemp.mp3");
    }

    etapaActual = ETAPA_TEMP;
    tempSumAuto = 0;
    tempCountAuto = 0;
    tempPresente = false;

  } else if (etapaActual == ETAPA_TEMP) {
    if (!tempCapturada) {
      tempFinalAuto = (tempCountAuto > 0) ? (tempSumAuto / tempCountAuto) : temp;
      tempCapturada = true;
      encolarAudio("/a5_tempok.mp3");
    }

    etapaActual = ETAPA_FINAL;
    slideActual = 2; // Temperatura fue el ultimo parametro medido
    actualizarSemaforoParaSlideActual();

    String estadoFinal = obtenerEstado(bpmFinalAuto, spo2FinalAuto, tempFinalAuto);
    if (estadoFinal == "Alerta") {
      encolarAudio("/a8_alerta.mp3");
    } else if (estadoFinal == "Advertencia") {
      encolarAudio("/a7_atencion.mp3");
    } else {
      encolarAudio("/a6_estable.mp3");
    }
    // Despedida encadenada justo despues de dictar el resultado
    encolarAudio("/a9_despedida.mp3");
  }
}

void reiniciarBufferSPO2(); // forward declaration, definida mas abajo

void reiniciarTodo() {
  etapaActual = ETAPA_BPM_SPO2;
  slideActual = 0;

  dedoPuesto = false;
  ultimoBpmValido = 0;
  ultimoSpo2Valido = 0;
  bpmCapturado = false;
  spo2Capturado = false;
  bpmFinalAuto = 0;
  spo2FinalAuto = 0;

  tempPresente = false;
  tempSumAuto = 0;
  tempCountAuto = 0;
  tempCapturada = false;
  tempFinalAuto = 0;

  bpm = 0;
  bpmProm = 0;
  rateSpot = 0;
  bpmContador = 0;
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
  reiniciarBufferSPO2();

  apagarSemaforo();

  limpiarColaAudio();
  encolarAudio("/a1_saludo.mp3");
  encolarAudio("/a2_indic1.mp3");
}

void manejarBotones() {
  bool refrescarYa = false;

  bool lecturaSiguiente = digitalRead(BTN_SIGUIENTE);
  if (lecturaSiguiente != ultimaLecturaSiguiente) {
    ultimoDebounceSiguiente = millis();
  }
  if ((millis() - ultimoDebounceSiguiente) > DEBOUNCE_DELAY) {
    if (lecturaSiguiente != estadoSiguiente) {
      estadoSiguiente = lecturaSiguiente;
      if (estadoSiguiente == LOW) {
        if (etapaActual == ETAPA_FINAL) {
          slideActual = (slideActual + 1) % TOTAL_SLIDES;
          actualizarSemaforoParaSlideActual();
        } else {
          avanzarEtapa();
        }
        refrescarYa = true;
      }
    }
  }
  ultimaLecturaSiguiente = lecturaSiguiente;

  bool lecturaReiniciar = digitalRead(BTN_REINICIAR);
  if (lecturaReiniciar != ultimaLecturaReiniciar) {
    ultimoDebounceReiniciar = millis();
  }
  if ((millis() - ultimoDebounceReiniciar) > DEBOUNCE_DELAY) {
    if (lecturaReiniciar != estadoReiniciar) {
      estadoReiniciar = lecturaReiniciar;
      if (estadoReiniciar == LOW) {
        reiniciarTodo();
        refrescarYa = true;
      }
    }
  }
  ultimaLecturaReiniciar = lecturaReiniciar;

  if (refrescarYa) {
    actualizarOLED();
  }
}

void actualizarOLED() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  switch (etapaActual) {
    case ETAPA_BPM_SPO2:
      mostrarEtapaBPMSpO2();
      break;

    case ETAPA_TEMP:
      mostrarEtapaTemp();
      break;

    case ETAPA_FINAL:
      switch (slideActual) {
        case 0: mostrarSlideBPM(); break;
        case 1: mostrarSlideSpO2(); break;
        case 2: mostrarSlideTemp(); break;
      }
      break;
  }

  display.display();
}

void actualizarCapturaBPMSpO2() {
  bool dedoAhora = irValue >= 20000;
  const unsigned long TIEMPO_CONTACTO_REQUERIDO = 30000; // 30 segundos de contacto estable

  if (dedoAhora) {
    if (!dedoPuesto) {
      dedoPuesto = true;
      dedoPuestoDesde = millis();
    }
    if (bpmProm > 0) ultimoBpmValido = bpmProm;
    if (spo2Actual > 0) ultimoSpo2Valido = spo2Actual;

    bool yaEstabaCapturado = bpmCapturado || spo2Capturado;

    // Apenas se cumplen los 30 segundos de contacto estable, guardamos y
    // avisamos por audio, sin necesidad de que retiren el dedo.
    if (!yaEstabaCapturado && (millis() - dedoPuestoDesde >= TIEMPO_CONTACTO_REQUERIDO)) {
      if (ultimoBpmValido > 0) {
        bpmFinalAuto = ultimoBpmValido;
        bpmCapturado = true;
      }
      if (ultimoSpo2Valido > 0) {
        spo2FinalAuto = ultimoSpo2Valido;
        spo2Capturado = true;
      }

      if (bpmCapturado || spo2Capturado) {
        encolarAudio("/a3_indic2.mp3");
        encolarAudio("/a4_medirtemp.mp3");
      }
    }
  } else {
    // Si retiran el dedo antes de completar el tiempo requerido, reiniciamos
    // el conteo para que el proximo contacto empiece limpio.
    if (!(bpmCapturado || spo2Capturado)) {
      ultimoBpmValido = 0;
      ultimoSpo2Valido = 0;
    }
    dedoPuesto = false;
  }
}

void actualizarCapturaTemp(bool presenteAhora) {
  const unsigned long TIEMPO_CONTACTO_REQUERIDO = 5000; // 5 segundos de contacto estable

  if (presenteAhora) {
    if (!tempPresente) {
      tempPresente = true;
      tempPresenteDesde = millis();
      tempSumAuto = 0;
      tempCountAuto = 0;
    }
    tempSumAuto += temp;
    tempCountAuto++;

    // Apenas se cumple el tiempo de contacto estable, guardamos y avisamos
    // por audio, sin necesidad de que retiren el sensor.
    if (!tempCapturada && (millis() - tempPresenteDesde >= TIEMPO_CONTACTO_REQUERIDO)) {
      tempFinalAuto = tempSumAuto / tempCountAuto;
      tempCapturada = true;
      encolarAudio("/a5_tempok.mp3");
    }
  } else {
    // Si retiran el sensor antes de completar el tiempo requerido, reiniciamos
    // el conteo para que el proximo contacto empiece limpio.
    if (!tempCapturada) {
      tempSumAuto = 0;
      tempCountAuto = 0;
    }
    tempPresente = false;
  }
}

void reiniciarBufferSPO2() {
  spo2BufferIndex = 0;
  spo2Actual = 0;
}

void handleDatos() {
  String json = "{";
  json += "\"bpm\":" + String(bpmProm) + ",";
  json += "\"spo2\":" + String(spo2Actual) + ",";
  json += "\"temp\":" + String(temp, 1) + ",";
  json += "\"tempAmb\":" + String(tempAmb, 1) + ",";
  json += "\"estado\":\"" + obtenerEstado(bpmProm, spo2Actual, temp) + "\",";
  json += "\"tempAuto\":" + String(tempFinalAuto, 1) + ",";
  json += "\"bpmAuto\":" + String(bpmFinalAuto) + ",";
  json += "\"spo2Auto\":" + String(spo2FinalAuto) + ",";
  json += "\"tempCapturada\":" + String(tempCapturada ? "true" : "false") + ",";
  json += "\"bpmCapturado\":" + String(bpmCapturado ? "true" : "false") + ",";
  json += "\"spo2Capturado\":" + String(spo2Capturado ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

void handleGuardarRegistro() {
  String nombre = server.arg("nombre");
  int edad = server.arg("edad").toInt();

  guardarRegistro(nombre, edad);
  server.send(200, "text/plain", "Registro guardado");
}

void handleHistorial() {
  String json = "[";
  for (int i = 0; i < totalRegistros; i++) {
    json += "{";
    json += "\"nombre\":\"" + historial[i].nombre + "\",";
    json += "\"edad\":" + String(historial[i].edad) + ",";
    json += "\"bpm\":" + String(historial[i].bpm) + ",";
    json += "\"spo2\":" + String(historial[i].spo2) + ",";
    json += "\"temp\":" + String(historial[i].temp, 1) + ",";
    json += "\"estado\":\"" + historial[i].estado + "\",";
    json += "\"chequeo\":\"" + historial[i].chequeo + "\"";
    json += "}";
    if (i < totalRegistros - 1) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleLimpiarHistorial() {
  totalRegistros = 0;
  server.send(200, "text/plain", "Historial limpiado");
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MedMonitor</title>

<style>
:root {
  --bg: #101820;
  --card: #1f2937;
  --header: #16213e;
  --text: white;
  --canvas: #0f172a;
}

.light {
  --bg: #f4f4f5;
  --card: white;
  --header: #dbeafe;
  --text: #111827;
  --canvas: #e5e7eb;
}

body {
  margin: 0;
  font-family: Arial, sans-serif;
  background: var(--bg);
  color: var(--text);
  text-align: center;
}

.header {
  background: var(--header);
  padding: 18px;
}

h1 {
  margin: 0;
  font-size: 30px;
}

.subtitle {
  font-size: 14px;
  color: #94a3b8;
}

.theme-switch {
  width: 120px;
  height: 60px;
  margin: 15px auto 0;
  background: #ff4d0d;
  border-radius: 50px;
  position: relative;
  cursor: pointer;
  transition: 0.4s;
}

.switch-slider {
  width: 48px;
  height: 48px;
  background: white;
  border-radius: 50%;
  position: absolute;
  top: 6px;
  left: 6px;
  display: flex;
  justify-content: center;
  align-items: center;
  font-size: 24px;
  transition: 0.4s;
}

.container {
  padding: 15px;
}

.card {
  background: var(--card);
  margin: 15px auto;
  padding: 18px;
  max-width: 430px;
  border-radius: 18px;
  box-shadow: 0 0 12px rgba(0,0,0,0.35);
}

.value {
  font-size: 36px;
  font-weight: bold;
}

.normal { color: #22c55e; }
.advertencia { color: #facc15; }
.alerta { color: #ef4444; }
.midiendo { color: #38bdf8; }

.ok { color: #22c55e; font-weight: bold; }
.wait { color: #facc15; font-weight: bold; }

.chart-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.fullscreen-btn {
  background: transparent;
  color: var(--text);
  border: 2px solid var(--text);
  border-radius: 8px;
  font-size: 22px;
  padding: 4px 10px;
  cursor: pointer;
}

#exitFullscreenBtn {
  display: none;
  position: fixed;
  top: 20px;
  right: 20px;
  width: 80px;
  height: 80px;
  border: 3px solid white;
  border-radius: 50%;
  background: #161625;
  color: white;
  font-size: 40px;
  cursor: pointer;
  z-index: 9999;
  box-shadow: 0 0 20px rgba(0,0,0,0.5);
}

input {
  padding: 12px;
  width: 85%;
  border-radius: 10px;
  border: none;
  font-size: 16px;
}

.btn {
  background: #22c55e;
  color: white;
  border: none;
  border-radius: 12px;
  padding: 12px 16px;
  margin: 5px;
  font-size: 15px;
  font-weight: bold;
  cursor: pointer;
}

.btn-danger {
  background: #ef4444;
}

table {
  width: 100%;
  font-size: 13px;
  border-collapse: collapse;
}

th, td {
  padding: 6px;
  border-bottom: 1px solid #64748b;
}

canvas {
  background: var(--canvas);
  border-radius: 12px;
  width: 100%;
  height: 220px;
}

.footer {
  font-size: 12px;
  color: #94a3b8;
  margin-top: 20px;
}
</style>
</head>

<body>
<div class="header">
  <h1>MedMonitor</h1>
  <div class="subtitle">Dashboard local ESP32</div>

  <div class="theme-switch" onclick="cambiarTema()">
    <div class="switch-slider" id="switchSlider">☀</div>
  </div>
</div>

<div class="container">

<div class="card">
  <h2>Datos actuales</h2>
  <div>BPM</div>
  <div id="bpmActual" class="value">--</div>
  <br>
  <div>SpO2</div>
  <div id="spo2Actual" class="value">-- %</div>
  <br>
  <div>Temperatura</div>
  <div id="tempActual" class="value">-- °C</div>
  <br>
  <div>Estado</div>
  <div id="estadoActual" class="value midiendo">Midiendo</div>
</div>

<div class="card">
  <h2>Captura automatica</h2>
  <p id="tempAutoText" class="wait">Temperatura: pendiente</p>
  <p id="bpmAutoText" class="wait">BPM: pendiente</p>
  <p id="spo2AutoText" class="wait">SpO2: pendiente</p>
  <p>Mantenga el dedo firme unos segundos hasta que se estabilice la lectura.</p>
</div>

<div class="card">
  <h2>Registro de Paciente</h2>

  <input id="nombrePaciente" placeholder="Nombre del paciente">
  <br><br>
  <input id="edadPaciente" type="number" placeholder="Edad">
  <br><br>

  <button class="btn" onclick="guardarRegistro()">Guardar medicion</button>
  <button class="btn btn-danger" onclick="limpiarHistorial()">Limpiar historial</button>

  <h3>Historial ultimos 10</h3>
  <div id="tablaHistorial">Sin registros</div>
</div>

<div class="card">
  <div class="chart-header">
    <h2>BPM vs Tiempo</h2>
    <button class="fullscreen-btn" onclick="fullscreenCanvas('bpmChart')">⛶</button>
  </div>
  <canvas id="bpmChart" width="400" height="220"></canvas>
</div>

<div class="card">
  <div class="chart-header">
    <h2>Temperatura vs Tiempo</h2>
    <button class="fullscreen-btn" onclick="fullscreenCanvas('tempChart')">⛶</button>
  </div>
  <canvas id="tempChart" width="400" height="220"></canvas>
</div>

<div class="card">
  <div class="chart-header">
    <h2>SpO2 vs Tiempo</h2>
    <button class="fullscreen-btn" onclick="fullscreenCanvas('spo2Chart')">⛶</button>
  </div>
  <canvas id="spo2Chart" width="400" height="220"></canvas>
</div>

<div class="footer">
  Actualizacion cada 1 segundo<br>
  Conexion local mediante ESP32 Access Point
</div>

</div>

<button id="exitFullscreenBtn" onclick="salirFullscreen()">✕</button>

<script>
let bpmData = [];
let tempData = [];
let spo2Data = [];
let maxPoints = 30;
let temaClaro = false;

function cambiarTema() {
  temaClaro = !temaClaro;

  const sw = document.getElementById("switchSlider");
  const bg = document.querySelector(".theme-switch");

  if (temaClaro) {
    document.body.classList.add("light");
    sw.style.left = "66px";
    sw.innerHTML = "🌙";
    bg.style.background = "#161625";
  } else {
    document.body.classList.remove("light");
    sw.style.left = "6px";
    sw.innerHTML = "☀";
    bg.style.background = "#ff4d0d";
  }

  dibujarGrafica("bpmChart", bpmData, 40, 120, "");
  dibujarGrafica("tempChart", tempData, 22, 42, "C");
  dibujarGrafica("spo2Chart", spo2Data, 70, 100, "%");
}

async function guardarRegistro() {
  let nombre = document.getElementById("nombrePaciente").value;
  let edad = document.getElementById("edadPaciente").value;

  await fetch("/guardar?nombre=" + encodeURIComponent(nombre) + "&edad=" + edad);
  cargarHistorial();
}

async function cargarHistorial() {
  const response = await fetch("/historial");
  const data = await response.json();

  if (data.length === 0) {
    document.getElementById("tablaHistorial").innerHTML = "Sin registros";
    return;
  }

  let html = "<table>";
  html += "<tr><th>Paciente</th><th>Edad</th><th>BPM</th><th>SpO2</th><th>Temp</th><th>Chequeo</th></tr>";

  data.forEach(r => {
    html += "<tr>";
    html += "<td>" + r.nombre + "</td>";
    html += "<td>" + r.edad + "</td>";
    html += "<td>" + r.bpm + "</td>";
    html += "<td>" + r.spo2 + "%</td>";
    html += "<td>" + r.temp.toFixed(1) + "C</td>";
    html += "<td class='" + colorChequeo(r.estado) + "'>" + (r.chequeo || r.estado) + "</td>";
    html += "</tr>";
  });

  html += "</table>";
  document.getElementById("tablaHistorial").innerHTML = html;
}

async function limpiarHistorial() {
  await fetch("/limpiar");
  cargarHistorial();
}

function fullscreenCanvas(id) {
  const canvas = document.getElementById(id);

  if (canvas.requestFullscreen) {
    canvas.requestFullscreen();
  } else if (canvas.webkitRequestFullscreen) {
    canvas.webkitRequestFullscreen();
  }

  document.getElementById("exitFullscreenBtn").style.display = "block";
}

function salirFullscreen() {
  if (document.fullscreenElement) {
    document.exitFullscreen();
  } else if (document.webkitFullscreenElement) {
    document.webkitExitFullscreen();
  }
}

document.addEventListener("fullscreenchange", function() {
  const btn = document.getElementById("exitFullscreenBtn");

  if (document.fullscreenElement) {
    btn.style.display = "block";
  } else {
    btn.style.display = "none";
  }
});

function claseEstado(estado) {
  if (estado === "Normal") return "value normal";
  if (estado === "Advertencia") return "value advertencia";
  if (estado === "Alerta") return "value alerta";
  return "value midiendo";
}

function colorChequeo(estado) {
  if (estado === "Normal") return "normal";
  if (estado === "Advertencia") return "advertencia";
  if (estado === "Alerta") return "alerta";
  return "midiendo";
}

function dibujarGrafica(canvasId, data, minY, maxY, unidad) {
  const canvas = document.getElementById(canvasId);
  const ctx = canvas.getContext("2d");

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const left = 45;
  const right = 390;
  const top = 20;
  const bottom = 190;

  ctx.fillStyle = temaClaro ? "#111827" : "#cbd5e1";
  ctx.font = "11px Arial";

  ctx.fillText(maxY + unidad, 5, top + 5);
  ctx.fillText(minY + unidad, 5, bottom);

  for (let i = 0; i < maxPoints; i++) {
    let x = left + i * ((right - left) / (maxPoints - 1));

    if (i % 5 === 0) {
      ctx.fillText(i + "s", x - 8, 210);
    }
  }

  ctx.strokeStyle = "#38bdf8";
  ctx.lineWidth = 2;
  ctx.beginPath();

  for (let i = 0; i < data.length; i++) {
    let x = left + i * ((right - left) / (maxPoints - 1));
    let value = data[i];
    let y = bottom - ((value - minY) / (maxY - minY)) * (bottom - top);

    if (y < top) y = top;
    if (y > bottom) y = bottom;

    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }

  ctx.stroke();
}

async function actualizarDatos() {
  try {
    const response = await fetch("/datos");
    const data = await response.json();

    document.getElementById("bpmActual").innerText = data.bpm > 0 ? data.bpm + " BPM" : "-- BPM";
    document.getElementById("spo2Actual").innerText = data.spo2 > 0 ? data.spo2 + " %" : "-- %";
    document.getElementById("tempActual").innerText = data.temp.toFixed(1) + " °C";

    let estado = document.getElementById("estadoActual");
    estado.innerText = data.estado;
    estado.className = claseEstado(data.estado);

    if (data.tempCapturada) {
      document.getElementById("tempAutoText").innerText = "Temperatura capturada: " + data.tempAuto.toFixed(1) + " °C";
      document.getElementById("tempAutoText").className = "ok";
    } else {
      document.getElementById("tempAutoText").innerText = "Temperatura: pendiente";
      document.getElementById("tempAutoText").className = "wait";
    }

    if (data.bpmCapturado) {
      document.getElementById("bpmAutoText").innerText = "BPM capturado: " + data.bpmAuto;
      document.getElementById("bpmAutoText").className = "ok";
    } else {
      document.getElementById("bpmAutoText").innerText = "BPM: pendiente";
      document.getElementById("bpmAutoText").className = "wait";
    }

    if (data.spo2Capturado) {
      document.getElementById("spo2AutoText").innerText = "SpO2 capturado: " + data.spo2Auto + " %";
      document.getElementById("spo2AutoText").className = "ok";
    } else {
      document.getElementById("spo2AutoText").innerText = "SpO2: pendiente";
      document.getElementById("spo2AutoText").className = "wait";
    }

    bpmData.push(data.bpm);
    tempData.push(data.temp);
    spo2Data.push(data.spo2 > 0 ? data.spo2 : (spo2Data.length ? spo2Data[spo2Data.length - 1] : 100));

    if (bpmData.length > maxPoints) {
      bpmData.shift();
      tempData.shift();
      spo2Data.shift();
    }

    dibujarGrafica("bpmChart", bpmData, 40, 120, "");
    dibujarGrafica("tempChart", tempData, 22, 42, "C");
    dibujarGrafica("spo2Chart", spo2Data, 70, 100, "%");

  } catch (error) {
    console.log("Error leyendo datos:", error);
  }
}

setInterval(actualizarDatos, 1000);
actualizarDatos();
cargarHistorial();
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);

  // Bus 1: OLED + MLX90614
  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(100);

  // Bus 2: MAX30102
  I2C_PULSO.begin(SDA2_PIN, SCL2_PIN);
  I2C_PULSO.setClock(400000);
  delay(100);

  EEPROM.begin(EEPROM_SIZE);

  if (leerStringEEPROM(SSID1_ADDR) == "") {
    escribirStringEEPROM(SSID1_ADDR, "RedCasa");
    escribirStringEEPROM(PASS1_ADDR, "12345678");
    escribirStringEEPROM(SSID2_ADDR, "HotspotCel");
    escribirStringEEPROM(PASS2_ADDR, "87654321");
  }

  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_AMARILLO, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  apagarSemaforo();

  pinMode(BTN_SIGUIENTE, INPUT_PULLUP);
  pinMode(BTN_REINICIAR, INPUT_PULLUP);

  if (!display.begin(0x3C, true)) {
    Serial.println("ERROR: OLED SH1106 no responde (0x3C)");
    while (true);
  }

  if (!mlx.begin()) {
    Serial.println("ERROR: MLX90614 no responde (0x5A)");
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("ERROR SENSOR");
    display.println("MLX90614 (0x5A)");
    display.println("no responde.");
    display.println("Revisar cableado");
    display.println("SDA=21 SCL=22");
    display.display();
    while (true);
  }

  bool maxOk = false;
  for (int intento = 1; intento <= 5 && !maxOk; intento++) {
    maxOk = particleSensor.begin(I2C_PULSO, I2C_SPEED_FAST);
    if (!maxOk) {
      Serial.print("MAX30102 intento ");
      Serial.print(intento);
      Serial.println(" fallido, reintentando...");
      delay(300);
    }
  }

  if (!maxOk) {
    Serial.println("ERROR: MAX30102 no responde (0x57)");
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("ERROR SENSOR");
    display.println("MAX30102 (0x57)");
    display.println("no responde.");
    display.println("Revisar cableado");
    display.println("SDA=32 SCL=33");
    display.display();
    while (true);
  }
  Serial.println("MAX30102 OK (0x57)");

  particleSensor.setup(0x0F, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x0F);
  particleSensor.setPulseAmplitudeIR(0x0F);
  particleSensor.setPulseAmplitudeGreen(0);

  // --- Audio (MAX98357A por I2S) ---
  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: no se pudo montar LittleFS (revisar que se haya subido el filesystem con 'Upload Filesystem Image')");
  }
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(15); // rango 0-21

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password, 6, 0, 4);

  server.on("/", handleRoot);
  server.on("/datos", handleDatos);
  server.on("/guardar", handleGuardarRegistro);
  server.on("/historial", handleHistorial);
  server.on("/limpiar", handleLimpiarHistorial);
  server.begin();

  // Saludo inicial + instruccion de la primera etapa
  encolarAudio("/a1_saludo.mp3");
  encolarAudio("/a2_indic1.mp3");
}

void loop() {
  server.handleClient();

  // Mantiene el audio sonando y dispara el siguiente de la cola cuando el
  // actual termina. Debe llamarse seguido, sin bloquear.
  audio.loop();
  procesarColaAudio();
  actualizarVolumen();

  // 1. Leer temperatura y aplicar compensacion
  float tempPiel = mlx.readObjectTempC();
  tempAmb = mlx.readAmbientTempC();
  bool objetoTempPresente = (tempPiel >= 27.0 && tempPiel <= 35.5);

  if (objetoTempPresente) {
    temp = tempPiel + (0.30 * (37.0 - tempAmb));
  } else {
    temp = tempPiel;
  }

  // 2. Extraer datos del sensor MAX30105
  particleSensor.check();
  while (particleSensor.available()) {
    irValue = particleSensor.getFIFOIR();
    uint32_t redValue = particleSensor.getFIFORed();
    particleSensor.nextSample();

    if (checkForBeat(irValue)) {
      heartBig = !heartBig;
      long now = millis();
      long delta = now - lastBeat;
      lastBeat = now;
      float bpmInstantaneo = 60.0 / (delta / 1000.0);

      if (bpmInstantaneo > 45 && bpmInstantaneo < 200) {
        bpm = bpmInstantaneo;
        rates[rateSpot++] = (byte)bpm;
        rateSpot %= RATE_SIZE;
        if (bpmContador < RATE_SIZE) bpmContador++;

        int suma = 0;
        for (byte i = 0; i < bpmContador; i++) suma += rates[i];
        bpmProm = suma / bpmContador;
      }
    }

    irBuffer[spo2BufferIndex] = irValue;
    redBuffer[spo2BufferIndex] = redValue;
    spo2BufferIndex++;

    if (spo2BufferIndex >= SPO2_BUFFER_SIZE) {
      maxim_heart_rate_and_oxygen_saturation(irBuffer, SPO2_BUFFER_SIZE, redBuffer,
                                             &spo2Calculado, &spo2Valido,
                                             &hrCalculado, &hrValidoAlgoritmo);
      if (spo2Valido && spo2Calculado > 0 && spo2Calculado <= 100) {
        spo2Actual = spo2Calculado;
      }

      for (int i = 25; i < SPO2_BUFFER_SIZE; i++) {
        irBuffer[i - 25] = irBuffer[i];
        redBuffer[i - 25] = redBuffer[i];
      }
      spo2BufferIndex = SPO2_BUFFER_SIZE - 25;
    }
  }

  // 3. Captura automatica: solo corre la logica de la etapa actual
  if (etapaActual == ETAPA_BPM_SPO2) {
    actualizarCapturaBPMSpO2();
  } else if (etapaActual == ETAPA_TEMP) {
    actualizarCapturaTemp(objetoTempPresente);
  }

  // 4. Si no hay dedo en el sensor de pulso, reiniciamos sus buffers
  if (irValue < 20000) {
    bpm = 0;
    bpmProm = 0;
    rateSpot = 0;
    bpmContador = 0;
    for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
    reiniciarBufferSPO2();
  }

  // 5. Botones
  manejarBotones();

  // Mientras se esta midiendo, refrescamos mas seguido para que la
  // animacion se vea fluida; en resultados alcanza con el intervalo normal.
  unsigned long intervaloActual = (etapaActual == ETAPA_FINAL) ? DISPLAY_INTERVAL : 150;

  if (millis() - lastDisplayUpdate >= intervaloActual) {
    lastDisplayUpdate = millis();
    actualizarOLED();
  }

  // Segunda llamada, para que el audio se sirva mas seguido y no se corte
  // por el trabajo del OLED/sensores/WiFi de esta vuelta del loop.
  audio.loop();

  delay(1);
}

// Callback requerido por la libreria de audio (opcional, solo para debug)
void audio_info(const char *info) {
  Serial.print("audio_info: ");
  Serial.println(info);
}