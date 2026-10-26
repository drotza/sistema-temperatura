# Sistema de control de temperatura

Control en lazo cerrado (PI/PID) de la temperatura de un cartucho calefactor con Arduino Uno.
La referencia se cambia desde una interfaz en Python que grafica en vivo y guarda los datos en CSV.

Todo trabaja en **unidades crudas**:
- temperatura y referencia en cuentas del ADC (0–1023);
- salida en cuentas de PWM (0–255).

## Contenido

| Archivo | Qué es |
|---|---|
| `PIArdController/PIArdController.ino` | Sketch del Arduino. PI/PID con anti-windup y habilitación del calentador (salida = control AND habilitado) |
| `Python/control_gui.py` | Interfaz: gráfica de temperatura y PWM, perilla de referencia, ganancias, lazo abierto/cerrado, CSV + PNG |
| `Python/knob.png` | Imagen de la perilla (ImageKnob de TkDial) |
| `Python/on.png`, `Python/off.png` | Imágenes del switch del calentador |
| `requirements.txt` | Dependencias de Python |

## Conexiones

| Desde | Hacia |
|---|---|
| D3 (PWM) | R1 1 kΩ → TIP121 base (pata 1) |
| TIP121 colector (pata 2) | Calentador → Pila 9 V (+) |
| TIP121 emisor (pata 3) | GND común |
| 5 V | Termistor NTC 10k B3950 → A0 |
| A0 | R2 1 kΩ → GND |
| Pila 9 V (−) | GND del Arduino (tierra común) |

Notas:
- Con el termistor arriba del divisor, **más temperatura = más cuentas**.
- El TIP121 se calienta; vigílalo en corridas largas o ponle disipador (la pestaña está conectada al colector).
- La corriente del calentador no debe pasar por las tiras de la protoboard (aguantan ~1 A).

## Uso

1. Cargar `PIArdController/PIArdController.ino` en el Arduino Uno (cerrar el Monitor Serial después).
2. Instalar dependencias y correr la interfaz desde la carpeta `Python`:

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
cd Python
python control_gui.py COM6            # puerto del Arduino
python control_gui.py FAKE            # planta simulada en tiempo real, sin Arduino
python control_gui.py FAKE 10         # simulada, 10x más rápido
python control_gui.py REPLAY [x.csv]  # reproduce una corrida guardada (calentador apagado)
python control_gui.py VACIA           # solo abre la interfaz, sin datos (no guarda nada)
```

- **FAKE** simula la planta térmica sin Arduino.
- **REPLAY** reproduce en tiempo real una corrida guardada en CSV, a partir del tramo ya estable
  después de apagar el calentador.

En la ventana:
- **Switch ON/OFF** del calentador: habilita/apaga la salida (arranca en OFF).
- **Perilla** "Temp Deseada": referencia en cuentas de ADC (rueda o arrastrar).
- **Lazo cerrado (PID) / Lazo abierto**: compara el control contra un PWM fijo.
- **Kp, Ki, Kd + Aplicar**: cambia ganancias sin salto en la salida.
- **Marcar perturbación**: deja una marca en la gráfica.
- Al cerrar se apaga el calentador y se guardan `CSV_PNG/pid_<fecha>.csv` (`t,PWM,Temp`) y su `.png`.

## Protocolo serial (115200 baud)

| Orden | Acción |
|---|---|
| `L` | Empieza a enviar datos |
| `R<adc>` | Referencia en cuentas de ADC |
| `K<kp>,<ki>,<kd>` | Ganancias del controlador |
| `A` | Lazo cerrado (PID) |
| `M<pwm>` | Lazo abierto con PWM fijo 0–255 |
| `E<0\|1>` | Habilita / apaga el calentador |
| `X` | Apaga y deja de enviar |

Cada 0.5 s el Arduino envía `t_s,ref_adc,adc,pwm,modo,hab`.
