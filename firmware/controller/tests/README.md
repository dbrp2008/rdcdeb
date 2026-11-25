# Joystick DMA watchdog

Run the host-side state-machine test from the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/controller/Core/Inc \
  firmware/controller/tests/test_joystick_dma_watchdog.c -o /tmp/test_joystick_dma_watchdog
/tmp/test_joystick_dma_watchdog
```

The controller polls the DMA channel 1 transfer-complete and transfer-error flags.
It sends zero translation/rotation until a full scan has completed, and latches a
fault on startup failure, a transfer error, disabled ADC/DMA, or no observed scan
progress for more than 200 ms. A latched fault requires a controller restart. The
mechanism deadman remains independent. This is a software motion-command inhibit,
not a motor power cutoff. The mainboard's averaging and closed-loop behavior can
delay the response to zero commands.

DMA activity does not verify the joystick voltages, detect a disconnected or
stuck joystick, or validate individual channel freshness. A foreground stall can
also delay this watchdog; the mainboard link timeout remains important. Bench
check the implementation with motor power isolated, including disconnecting or
stopping DMA, checking zero UART motion lines and the ADC screen status, then
verifying the full system with the wheels lifted and a separate emergency stop.

Test the mechanism-button truth table and its encoded command lines:

```sh
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/controller/Core/Inc -Ifirmware/common \
  firmware/controller/tests/test_mechanism_mapping.c firmware/common/rdc_protocol.c \
  -o /tmp/test_mechanism_mapping
/tmp/test_mechanism_mapping
```

Test turn button chord recognition and one-shot behavior:

```sh
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/controller/Core/Inc -Ifirmware/common \
  firmware/controller/tests/test_turn_buttons.c -o /tmp/test_turn_buttons
/tmp/test_turn_buttons
```
