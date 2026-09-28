# Guía de estudio — Induction Motor VFD Simulator

Este documento explica, paso a paso y en español, la teoría que implementa el simulador, las
decisiones de diseño que se tomaron (y las alternativas que se descartaron) y un banco de
preguntas de entrevista técnica. Crece con cada fase del proyecto; esta versión cubre la
**Fase 1: planta y control V/f**.

Los números que se citan provienen de `docs/results/phase1.md`, generado por
`vfdsim experiments phase1`.

---

## 1. Teoría paso a paso

### 1.1 Del sistema trifásico al vector espacial

Un motor trifásico tiene tres corrientes de fase $i_a, i_b, i_c$ que, con neutro aislado,
suman cero. Solo hay **dos grados de libertad**, así que se pueden representar como un vector en
el plano, el **vector espacial**:

$$
\underline{i} = i_\alpha + j\,i_\beta,\qquad
i_\alpha = \tfrac{2}{3}\left(i_a - \tfrac12 i_b - \tfrac12 i_c\right),\quad
i_\beta = \tfrac{1}{\sqrt3}(i_b - i_c).
$$

Es la **transformada de Clarke** en su versión *invariante en amplitud* (factor 2/3): si las
corrientes son senoidales equilibradas de amplitud $\hat I$, el vector tiene longitud $\hat I$ y
gira a la velocidad eléctrica $\omega_e$. El precio de esta convención es que la potencia no se
conserva directamente: $p = \tfrac32(v_\alpha i_\alpha + v_\beta i_\beta)$. En el código
(`src/vfdsim/transforms.py`) está verificado con tests.

La **transformada de Park** gira ese vector un ángulo $-\theta$ para verlo desde un sistema de
ejes que rota ($dq$). Si $\theta$ sigue al vector, las magnitudes senoidales se vuelven
constantes: es la base del control vectorial (Fase 3).

### 1.2 Modelo dinámico del motor jaula de ardilla

El rotor jaula de ardilla es un devanado trifásico cortocircuitado. Con los **flujos
concatenados** como variables de estado y todo referido al estator:

$$
\frac{d\underline\psi_s}{dt} = \underline v_s - R_s\underline i_s,\qquad
\frac{d\underline\psi_r}{dt} = -R_r\underline i_r + j\omega_r\underline\psi_r,
$$

$$
\underline\psi_s = L_s\underline i_s + L_m\underline i_r,\qquad
\underline\psi_r = L_m\underline i_s + L_r\underline i_r .
$$

* La primera ecuación es la ley de Faraday en el estator: la tensión aplicada menos la caída
  resistiva cambia el flujo.
* La segunda es lo mismo en el rotor (tensión cero, porque está en cortocircuito), más un término
  $j\omega_r\underline\psi_r$ que aparece porque escribimos el rotor, que gira, en ejes fijos.
* Las corrientes se obtienen invirtiendo la matriz de inductancias.

El par es el producto vectorial flujo × corriente:
$T_e = \tfrac32 p(\psi_{s\alpha}i_{s\beta} - \psi_{s\beta}i_{s\alpha})$, y la mecánica es la
segunda ley de Newton rotacional: $J\dot\omega_m = T_e - T_L - B\omega_m$.

### 1.3 Régimen permanente: circuito equivalente

Si la alimentación es senoidal y la velocidad constante, el modelo dq tiene una solución
senoidal que coincide exactamente con el **circuito equivalente en T** por fase
($R_s$, $X_{ls}$, $X_m$, $X_{lr}$, $R_r/s$). El simulador lo usa como *referencia independiente*:
el test `test_torque_speed_curve_matches_equivalent_circuit` bloquea el eje del modelo dinámico
en varias velocidades y comprueba que el par medio coincide con el del circuito (error máximo
$1.3\times10^{-5}$ N·m en la corrida de esta fase).

Relaciones clave:

* Velocidad síncrona: $n_s = 120 f / P$ (r/min), con $P$ el número de polos.
* Deslizamiento: $s = (n_s - n)/n_s$.
* Flujo de potencia: $P_{in} = P_{cu,s} + P_{ag}$; $P_{cu,r} = s\,P_{ag}$;
  $P_{mec} = (1-s)P_{ag}$; $T_e = P_{ag}/(\omega_s/p)$.
* Par máximo (de ruptura) con el equivalente de Thevenin: no depende de $R_r$; lo que depende
  de $R_r$ es el deslizamiento al que ocurre.

### 1.4 Control escalar V/f

Si se desprecia $R_s$, $\hat V_s \approx \omega_e\hat\psi_s$: **mantener V/f constante mantiene
el flujo**, y con el flujo constante la curva par-velocidad se desplaza horizontalmente al cambiar
la frecuencia sin cambiar de forma. Tres problemas prácticos y sus soluciones:

1. **Baja frecuencia** → la caída $R_s i_s$ deja de ser despreciable, el flujo cae y el motor
   pierde par. Solución: **boost** (tensión adicional $V_0$ a 0 Hz). En la corrida de esta fase,
   con 50 % de carga y sin boost el motor se detiene por debajo de ~5 Hz; con 6 % de boost gira
   a 2 Hz. Contrapartida: sobreexcitación (el flujo a 2 Hz sube a 0.614 Wb frente a 0.450 Wb
   nominales), que en un motor real produciría saturación y más corriente magnetizante.
2. **Deslizamiento** → a carga nominal el rotor gira más lento que el campo. Solución:
   **compensación de deslizamiento**: estimar el par a partir de la potencia
   ($\hat T = p(P_{in} - P_{cu,s})/\omega_e$) y sumar a la frecuencia la de deslizamiento
   correspondiente. A 30 Hz y par nominal el error pasa de 13.43 % a 1.03 %.
3. **Arranques bruscos** → corriente excesiva. Solución: **rampa** de frecuencia y **prevención
   de bloqueo** (la rampa se congela mientras la corriente filtrada supera el límite).

### 1.5 Modulación senoidal (SPWM) y límite de tensión

En cada rama del inversor la referencia se compara con una portadora triangular. En promedio
sobre un periodo de conmutación, la tensión del polo es $d_k V_{dc}$ con
$d_k = \tfrac12 + v_k^*/V_{dc}$. La zona lineal termina cuando la amplitud de fase llega a
$V_{dc}/2$, es decir, una tensión de línea eficaz de $0.612\,V_{dc}$. Con un bus de 285 V
(rectificador de 208 V) eso son 174.5 V < 208 V: **el variador con SPWM no puede dar la
tensión nominal**, la ley V/f se satura en 49.75 Hz y a 60 Hz el motor queda subexcitado. La
SVPWM (Fase 2) sube el límite un 15.5 % ($2/\sqrt3$).

### 1.6 El motor de simulación como un microcontrolador

El controlador se ejecuta cada $T_s = 100\,\mu s$: lee sensores, calcula, y el ciclo de trabajo
se aplica **en el periodo siguiente** (retardo de cómputo de una muestra, como un MCU que
actualiza los registros PWM para la siguiente portadora). Entre muestras, la planta continua se
integra con Runge-Kutta de orden 4. Consecuencia no evidente: el vector de tensión realmente
aplicado va retrasado $(0.5 + d)\,\omega_e T_s$ respecto del comando (retención de orden cero +
retardo), y el estimador de potencia de la compensación de deslizamiento lo corrige.

---

## 2. Decisiones de diseño y alternativas descartadas

| Decisión | Alternativas descartadas | Motivo |
|---|---|---|
| Flujos concatenados como estados, en ejes estacionarios | Corrientes como estados; ejes síncronos | Con flujos las ecuaciones son directas (Faraday) y sin derivadas de inductancias. Los ejes estacionarios permiten aplicar la tensión del inversor sin conocer ningún ángulo, algo imprescindible para el inversor conmutado de la Fase 2. |
| RK4 de paso fijo, sincronizado con el periodo de control | `scipy.integrate.solve_ivp` de paso variable | El controlador es discreto y actúa en instantes fijos; un integrador de paso variable tendría que detenerse en cada muestra y complicaría el inversor conmutado. RK4 con listas de Python es rápido para 12 estados (3 s simulados en ~1 s). |
| Acumuladores de energía integrados como estados | Calcular potencias desde las formas de onda muestreadas | Permite verificar la conservación de la energía con precisión de integración (residuo relativo ~$10^{-7}$), no con el error de muestreo. |
| Inversor promediado en la Fase 1 | Modelo conmutado desde el inicio | El promedio es el modelo estándar para diseñar el control y aísla los fenómenos. La interfaz por *segmentos* ya está preparada para el modelo conmutado. |
| Parámetros en YAML con procedencia (`assumed`/`measured`) y validación estricta | Constantes en el código; `pydantic` | Separar datos de código y hacer visible qué es supuesto era un requisito. Las `dataclasses` con validación propia evitan una dependencia pesada; se rechazan claves desconocidas para atrapar errores tipográficos. |
| Cargas como componentes sumables con signo suavizado ($\tanh$) | `sign()` discontinuo; modelo de fricción estática con estados | Un `sign` discontinuo hace oscilar al integrador en velocidad cero; la función suave mantiene el lado derecho continuo. Se documenta el pequeño *creep* que introduce. |
| Compensación de deslizamiento basada en potencia, con placa de características | Estimar deslizamiento con el modelo completo del motor | Es lo que hacen los variadores V/f industriales con solo datos de placa. La versión basada en modelo sería FOC encubierto, que llega en la Fase 3. |
| La compensación no puede superar la frecuencia máxima configurada | Permitir superarla | La frecuencia máxima protege la mecánica; es un límite duro en los variadores reales. |
| Parámetros supuestos autoconsistentes con la placa | Tomar valores "de tabla" sin verificar | Un circuito equivalente arbitrario no reproduce la velocidad nominal; aquí el circuito entrega 175 W a 1670 r/min con 208 V/60 Hz (verificado por un test). |

---

## 3. Preguntas de entrevista técnica (Fase 1)

**1. ¿Por qué la velocidad de un motor de inducción nunca alcanza la síncrona cuando tiene carga?**
Porque el par se produce por las corrientes inducidas en el rotor, y esas corrientes solo existen
si hay movimiento relativo entre el campo y el rotor. A velocidad síncrona no hay FEM inducida en
el rotor, ni corriente, ni par. El deslizamiento necesario crece con el par; en la zona estable
es aproximadamente proporcional a $T R_r/\psi^2$.

**2. ¿Qué pasa con el par máximo si se duplica la resistencia del rotor?**
El valor del par máximo no cambia (depende del equivalente de Thevenin y de la reactancia de
dispersión, no de $R_r$), pero ocurre a un deslizamiento el doble de grande. Por eso los motores
de rotor bobinado con resistencias externas aumentan el par de arranque sin cambiar el par máximo.

**3. ¿Por qué se mantiene V/f constante y por qué falla esa regla a baja frecuencia?**
Porque $\hat\psi_s \approx \hat V_s/\omega_e$ cuando se desprecia $R_s$, y un flujo constante
mantiene la capacidad de par. A baja frecuencia la tensión es pequeña y la caída $R_s i_s$ se
come una fracción importante; el flujo cae y el motor pierde par. Se corrige con *boost* o con
compensación IR.

**4. Un variador alimentado desde una red de 208 V con modulación SPWM, ¿puede entregar 208 V al
motor? ¿Qué consecuencias tiene?**
No. El bus DC de un rectificador de diodos queda cerca de $\sqrt2\cdot208\approx294$ V en vacío
(285 V supuestos con carga), y la SPWM lineal solo entrega $0.612\,V_{dc}\approx174.5$ V de línea.
Por encima de ~50 Hz el motor trabaja subexcitado: más deslizamiento y menos par máximo (2.07×
el nominal a 60 Hz en la simulación, frente a 2.93× con tensión nominal). Soluciones: SVPWM o
inyección de tercer armónico (+15.5 %), sobremodulación, o aceptar el debilitamiento de campo.

**5. ¿Cómo verificaría que un simulador de motor está bien implementado?**
Contra principios físicos y contra un modelo independiente: velocidad en vacío igual a
$120f/P$; par medio con el eje bloqueado igual al del circuito equivalente en toda la curva;
conservación de la energía (entrada = pérdidas + trabajo + variación de energía almacenada);
deslizamiento en régimen permanente igual al que predice el circuito equivalente. Los cuatro son
tests automáticos de este repositorio.

*(Las preguntas 6 a 10 se añadirán con las fases de potencia, control vectorial, control sin
sensor y núcleo embebido.)*
