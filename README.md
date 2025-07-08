# EV-Speed-Control-STM32

A speed control system for electric vehicles using an STM32 microcontroller, designed for precise real-time speed sensing and closed-loop control.

---

## 🛠️ Getting Started

### Requirements

- STM32CubeIDE (latest version recommended)
- STM32F407-DISC1 or compatible STM32 board
- 250W 24V BLDC motor 3000rpm (with Hall sensor)
- RMCS 3001 or Compatible Motor driver circuit 
- USB-UART converter (for debugging via serial)
- Suitable sensors for each parameter considered

For more on components used check out [COMPONENTS-LIST](./Documentation/IFP Components List.docx)

### Build & Flash

1. Clone this repository.
2. Open the project in STM32CubeIDE.
3. Connect your STM32 board and necessary peripherals.
4. Build the project and flash to target.

---

## ⚙️ Project Structure

- `Core/`: Application source code.
- `Drivers/`: STM32 HAL and peripheral drivers.
- `.ioc`: STM32CubeMX project configuration file.
- `.gitignore` and `.gitattributes`: Clean version control setup.
- `COMPONENTS-LIST.docx`: Component details.
- `README.md`: Project documentation.
- `LICENSE`: MIT License.

