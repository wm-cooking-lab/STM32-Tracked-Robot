# STM32 Robot – PI Speed Control & Bluetooth Interface

## Overview  
This project implements the embedded software controlling a pre-assembled STM32-based robot, as part of the *Conception de Systèmes Électroniques* course.  
The robot’s hardware — provided fully assembled — includes an STM32 Nucleo microcontroller, two DC motors with encoders, a Bluetooth module, and a battery monitoring circuit.  

All programming, debugging, and tuning were carried out independently, without direct guidance.  
The goal was to transform this hardware into a fully functional, safe, and remotely operated robot.



## Objectives  
According to the initial specifications, the robot must:
- Start or stop safely using a main switch (emergency stop)  
- Monitor the battery voltage and trigger alerts below 3 V  
- Move forward, backward, and rotate in both directions  
- Maintain precise PI-based speed control (±2 cm/s accuracy)  
- Allow three discrete speed levels: 10, 20, 30 cm/s  
- Receive commands wirelessly via Bluetooth  
- Stay within a 10 cm lateral deviation on a 3 m straight line  


## System Design  

### 1. Emergency Switch  
The robot uses the blue Nucleo push button (PC13) as a main interrupt-controlled switch.  
A software debounce filter ensures reliable operation and prevents false triggers.

### 2. Battery Monitoring  
Battery voltage (on PC5) is monitored using the Analog Watchdog peripheral of the ADC.  
When voltage < 3 V:
- The green LED turns on  
- A UART message is sent to the terminal  

### 3. PI Speed Control  
A Proportional–Integral (PI) controller regulates each motor’s speed using encoder feedback:
- Sampling period: 100 ms (TIM6 interrupt)
- PWM frequency: 500 Hz
- Parameters tuned experimentally (Kp, Ki) for each side

The controller computes:
 ```text
ε = v_ref - v_meas
PWM = Kp * ε + Ki * ∑ε
````
and adjusts the PWM duty cycle of each motor accordingly.

#### Variable definitions:
| Symbol / Variable | Description |
|-------------------|-------------|
| `v_ref` | Reference speed (desired speed), set via Bluetooth command |
| `v_meas` | Measured speed of the motor, obtained from the encoder |
| `ε` | Instantaneous error between reference and measured speed (`v_ref - v_meas`) |
| `Kp` | Proportional gain, applies an immediate correction proportional to the error |
| `Ki` | Integral gain, compensates for accumulated steady-state error over time |
| `∑ε` | Accumulated error (integral term) since the start of motion |
| `PWM` | Control output that adjusts the motor’s speed (PWM duty cycle) |

The controller updates the PWM output every **100 ms** using a TIM6 interrupt.  
PWM frequency is set to **500 Hz**, and gains `Kp` and `Ki` were determined experimentally to ensure smooth response and stable tracking across all speed levels (10, 20, 30 cm/s).

### 4. Bluetooth Remote Control  
The robot receives movement commands (A, R, G, D) via UART3 (Bluetooth module).  
Each command transitions the robot through a finite-state machine (FSM) managing:
- Forward / backward movement  
- Left / right rotations  
- Speed levels (1×, 2×, 3× base speed)  

## Control Logic Overview

The following algorith diagram summarizes the firmware behavior, including
initialization, PI control loop, safety callbacks, and state transitions:

![Algorithm Diagram](algorigramme_projet10.svg)


## Implementation Summary  

| Module | Function | MCU Resources |
|:--------|:----------|:---------------|
| GPIO | LED, direction pins, start button | PC13, PA5, PB2, PC8 |
| ADC + Analog Watchdog | Battery voltage monitoring | ADC1, PC5 |
| TIM2 (PWM) | Motor control | Channels 1 & 4 |
| TIM3 / TIM4 (Encoders) | Feedback speed measurement | TIM3 & TIM4 |
| TIM6 (Interrupt) | PI sampling (100 ms) | TIM6 |
| USART2 | Serial terminal output | 115200 bps |
| USART3 | Bluetooth commands | 9600 bps |



## Results  

| Test | Observation |
|:------|:-------------|
| PWM 10 cm/s | 30 % duty cycle |
| PWM 20 cm/s | 60 % duty cycle |
| PWM 30 cm/s | ≈ 100 % duty cycle |
| Straight motion | < 3 cm lateral deviation |
| Low battery alert | LED + UART message |
| Start/stop safety | Stable, no unintended motion |

All functional requirements were met.  
The PI controller provided smooth, stable motion and quick response to command changes.  



## Possible Improvements  
- Implement a closed-loop yaw control using a gyroscope for more precise turns  
- Add auto-calibration of motor constants  
- Optimize power management and include battery percentage estimation  
- Add obstacle detection sensors  


## Project Structure

The controller computes:

 ```text
STM32_Robot/
│
├── Core/
│ ├── Inc/
│ │ ├── main.h
│ │ ├── stm32l4xx_it.h
│ │ └── stm32l4xx_hal_conf.h
│ │
│ └── Src/
│ ├── main.c                # Main control logic, initialization, PI loop
│ ├── stm32l4xx_it.c        # Interrupt handlers
│ ├── stm32l4xx_hal_msp.c   # Hardware peripheral setup (MSP init)
│ ├── syscalls.c            # System call stubs (I/O redirection)
│ ├── sysmem.c              # Dynamic memory management (heap)
│ └── system_stm32l4xx.c    # System clock and MCU setup
│
├── startup/
│ └── startup_stm32l476rgtx.s
│
├── STM32L476RGTX_FLASH.ld
├── STM32L476RGTX_RAM.ld
├── robot10.ioc
├── .gitignore
└── README.md
```


## Build Instructions
1. Open the `.ioc` file in STM32CubeIDE  
2. Generate the code (`Project → Generate Code`)  
3. Build the project (`Ctrl + B`)  
4. Flash to your STM32 Nucleo-L476RG board  
5. Connect the Bluetooth module and serial terminal for testing


## Authors
**Walid Mouh Mouh**  
**Tao-Kann Martin**

---

## Build Instructions
1. Open the `.ioc` file in STM32CubeIDE  
2. Generate the code (`Project → Generate Code`)  
3. Build the project (`Ctrl + B`)  
4. Flash to your STM32 Nucleo-L476RG board  
5. Connect the Bluetooth module and serial terminal for testing

---

## Authors
**Walid Mouh Mouh**  
**Tao-kann Martin**

Autonomous development — École des Mines de Saint-Étienne  

