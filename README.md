# PIC16F18855 + Flexem HMI Starter (MPLAB XC8)

## What is implemented
- Internal oscillator initialization at **32 MHz**.
- GPIO setup per your pin layout.
- RS485 UART on **RC6 (TX)** and **RC7 (RX)**.
- **RB5 is not used** for DE/RE because RS485 module is auto RX/TX direction.
- Minimal Modbus RTU slave support:
  - **FC01** Read Coils (HMI communication check)
  - **FC05** Write Single Coil (HMI button control)
- Relay output control from HMI coils in manual mode:
  - Motor Forward/Reverse ON/OFF
  - Blower ON/OFF
  - Heater ON/OFF
- Communication watchdog:
  - RB4 relay goes ON when valid Modbus frame is received.
  - RB4 relay goes OFF on communication timeout.

## Coil address map (0x)
- 0000 Motor FWD Start
- 0001 Motor FWD Stop
- 0002 Motor REV Start
- 0003 Motor REV Stop
- 0004 Blower Start
- 0005 Blower Stop
- 0006 Heater Start
- 0007 Heater Stop
- 0010 Manual Mode
- 0011 Auto Mode

## Next step
- Add AUTO mode sequence logic step by step.
