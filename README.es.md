# Induction Motor VFD Simulator — resumen en español

[![CI](https://github.com/julianS-eng/induction-motor-vfd-sim/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/julianS-eng/induction-motor-vfd-sim/actions/workflows/ci.yml)
[![Licencia: MIT](https://img.shields.io/badge/licencia-MIT-blue.svg)](LICENSE)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-blue.svg)](pyproject.toml)

**Gemelo digital de un variador de frecuencia (VFD) que alimenta un motor de inducción jaula de
ardilla**: modelo dinámico dq de la máquina, cargas mecánicas, etapa de potencia, control escalar
V/f y, en fases posteriores, control vectorial (FOC), control sin sensor y un núcleo de control
en C ejecutado en *software-in-the-loop*.

La documentación técnica completa (teoría con ecuaciones, diagrama de bloques, resultados y
gráficas) está en inglés en el [README principal](README.md). La guía de estudio en español,
con la teoría paso a paso, las decisiones de diseño y preguntas de entrevista, está en
[docs/LEARNING.md](docs/LEARNING.md).

## Motivación

El proyecto acompaña una tesis de pregrado que es una **guía didáctica para implementar un
variador de velocidad en un motor jaula de ardilla con carga, usando equipos LabVolt / Festo**.
El simulador permite predecir cada experimento antes de ir al banco, explicar los fenómenos
(pérdida de par a baja velocidad, necesidad del *boost* y de la compensación de deslizamiento,
límite de tensión del modulador) y sintonizar controladores con la misma temporización que un
microcontrolador.

## Estado por fases

| Fase | Alcance | Estado |
|---|---|---|
| 1 | Modelo dq, cargas, inversor promediado con SPWM, V/f con rampa, boost y compensación de deslizamiento, tests físicos | ✅ terminada |
| 2 | Rectificador de diodos, bus DC, chopper de frenado, inversor conmutado con tiempos muertos, SVPWM, comparación SPWM vs SVPWM | ⏳ |
| 3 | FOC indirecto con lazos PI de corriente y velocidad, sintonía documentada, comparación arranque directo / V/f / FOC | ⏳ |
| 4 | FOC sin sensor con observador MRAS | ⏳ |
| 5 | Núcleo de control en C99 llamado con `ctypes` (SIL), notas de portabilidad a ESP32-S3 (MCPWM) | ⏳ |
| 6 | Identificación del circuito equivalente desde CSV, dashboard Streamlit, documentación final | ⏳ |

## Resultados principales de la Fase 1

Todos los números se generan con `vfdsim experiments phase1` y se inyectan en el README con
`scripts/sync_readme.py` (la CI verifica que estén sincronizados). Con el motor del laboratorio,
**LabVolt 8221-22** (200 W, 208 V, 60 Hz, 4 polos, 1685 r/min, 1,14 A, FP 0,73):

* **Validación del modelo**: la velocidad en vacío coincide con $n_s = 120 f / P$ (error relativo
  del orden de $10^{-8}$); el par del modelo dq con el eje bloqueado coincide con el circuito
  equivalente en 12 puntos con error máximo de $2.8\times10^{-7}$ N·m; el balance de potencia
  cierra con residuo relativo del orden de $10^{-7}$.
* **Límite del modulador**: con un bus de 285 V (puente de diodos desde 208 V) la SPWM entrega
  como máximo 174.5 V de línea, por lo que la ley V/f se satura en 49.25 Hz y a 60 Hz el par
  máximo cae a 1.41 veces el nominal (2.00× con tensión nominal).
* **Compensación de deslizamiento**: a 30 Hz y par nominal el error de velocidad baja de
  13.78 % a 0.48 %.
* **Boost**: por la resistencia de estator alta (≈ 20,7 Ω) se usa 12 % de boost. Con 30 % de
  carga y sin boost el motor se detiene hasta 8 Hz; con 12 % sigue girando a 2 Hz. Aun así, a
  15 Hz el par máximo es 0.95× el nominal: con este motor no conviene esperar par nominal en V/f
  por debajo de unos 20 Hz.

![Curva par-velocidad](docs/img/phase1_torque_speed.png)

## Uso rápido

```bash
pip install -e ".[dev]"
vfdsim info                      # datos del motor y punto nominal
vfdsim simulate --t-end 3        # escenario configs/default.yaml
vfdsim experiments phase1        # regenera figuras y tablas
pytest                           # tests
```

## Datos del motor

Esta rama (`labvolt-8221-22`) usa `configs/motors/labvolt_8221_22.yaml` con la **placa real** del
motor. El circuito equivalente se ajustó a la placa en modo motor con `scripts/fit_nameplate.py`
(Rs ≈ 20,65 Ω, Rr ≈ 8,19 Ω, Lls = Llr ≈ 21,6 mH, Lm ≈ 318,6 mH, suponiendo par máximo 2× nominal):
reproduce exactamente 200 W, 1685 r/min, 1,14 A y FP 0,73. Como validación, en modo alternador a
1900 r/min entrega 199,6 W pero con FP 0,40 y 1,38 A (placa: 0,47 y 1,18 A), diferencia atribuible
a que el modelo no tiene pérdidas en el hierro ni saturación. La inercia y la fricción siguen
siendo **supuestas**. Lo ideal es reemplazar el circuito por el identificado con los ensayos DC,
vacío y rotor bloqueado, y J/B con un ensayo de desaceleración libre. La rama `main` conserva el
motor genérico supuesto de 175 W.

## Relación con el banco LabVolt / Festo

Cada bloque del simulador corresponde a un módulo del banco: motor jaula de ardilla de cuatro
polos, fuente trifásica de 208 V, rectificador con condensadores de filtro, módulo
chopper/inversor IGBT, dinamómetro de cuatro cuadrantes (modos de par constante, ventilador,
fricción y volante de inercia) e interfaz de adquisición de datos. La tabla completa y el flujo
de trabajo sugerido para la guía están en la sección
[*Connection with the LabVolt / Festo bench*](README.md#connection-with-the-labvolt--festo-bench).

## Licencia

[MIT](LICENSE)
