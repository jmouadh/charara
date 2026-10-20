# Electrical

Hardware sources for the robot's electronics.

| Path | What it is |
|---|---|
| `Schematics/carte_deplacement_schematics.pdf` | Full schematic of the controller board: MCU, stepper drivers, power stage, connectors |
| `Routing/` | PCB routing captures, one per layer |
| `hw_real.jpeg` | Raw photo of the assembled board (kept out of git; the optimised copy is `../Docs/assets/hardware-electronics.jpg`) |

![The assembled controller board](../Docs/assets/hardware-electronics.jpg)

The board carries the STM32F722, four stepper drivers under heatsinks, the power
stage, the I2C and encoder connectors, and a status OLED. The firmware that runs
on it lives in [`../STM32/Firmware`](../STM32/Firmware); the wire protocol it
speaks is documented in
[`../Docs/STM_docs/03-i2c-protocol-spec.md`](../Docs/STM_docs/03-i2c-protocol-spec.md).
