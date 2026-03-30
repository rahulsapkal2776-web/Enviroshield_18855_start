# PIC16F18855 Starter (MPLAB XC8)

This starter project includes:
- Pin-wise mapping initialization for PIC16F18855.
- Flexem HMI Modbus coil mapping for manual mode commands.
- RS485 setup note for auto RX/TX module (no RB5 DE/RE control).

## File
- `main.c`: Basic firmware skeleton.

## Next steps
1. Add configuration bits and clock setup.
2. Initialize EUSART for Modbus RTU.
3. Integrate Modbus slave library and call `modbus_handle_coil_write()`.
4. Add safety interlocks and AUTO mode sequence logic.
