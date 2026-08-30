# Autonomous Breadboard Robot (Light Following & Obstacle Avoidance)

**Author:** Jacob Williams  
**Hardware Platform:** Arduino (ATmega328P), Breadboard Architecture, L298N H-Bridge Driver  
**Demonstration Video:** `robotProject.MOV` (Included in this folder)  
**Embedded Firmware:** `robot_controller.ino`, `pin_config.h`

---

## Architecture: Perception → Planning → Action
1. **Perception:** reads each button/pin and sets sensed-state variables (e.g., `SensedLightLeft`, `SensedLightUp`) on/off.
2. **Planning:** finite state machines decide behavior based on sensed state. Each state has explicit transition-in/out conditions — leaving a state requires a different input pattern than the one that entered it. Separate FSMs handle collision detection, steering, servo movement, and speed control.
3. **Action:** a switch statement (or small FSM) turns LEDs/motors/servo on or off based on the planning state.

---

## 📂 Included Project Files
1. **`robotProject.MOV`**: Demonstration video showing the breadboard robot in active operation.
2. **`robot_controller.ino`**: Embedded C++ firmware running the control loop and state machine.
3. **`pin_config.h`**: Hardware pin mapping and calibration constants.