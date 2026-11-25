# TR mainboard host tests

From the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/tr-mainboard/Core/Inc \
  firmware/tr-mainboard/tests/test_direction_config.c -o /tmp/test_tr_direction
/tmp/test_tr_direction
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/tr-mainboard/Core/Inc \
  firmware/tr-mainboard/tests/test_heading.c -lm -o /tmp/test_tr_heading
/tmp/test_tr_heading
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/tr-mainboard/Core/Inc -Ifirmware/common \
  firmware/tr-mainboard/tests/test_heading_integration.c firmware/common/rdc_protocol.c \
  -lm -o /tmp/test_tr_heading_integration
/tmp/test_tr_heading_integration
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/tr-mainboard/Core/Inc \
  firmware/tr-mainboard/tests/test_turn.c -lm -o /tmp/test_tr_turn
/tmp/test_tr_turn
cc -std=c11 -Wall -Wextra -Werror -Ifirmware/tr-mainboard/Core/Inc -Ifirmware/common \
  firmware/tr-mainboard/tests/test_turn_integration.c firmware/common/rdc_protocol.c \
  -lm -o /tmp/test_tr_turn_integration
/tmp/test_tr_turn_integration
```

The integration test compiles the actual drive source against mock HAL, CAN and
servo functions. It is not an STM32 firmware build or hardware test. Heading
geometry, sign, drive polarity, feedback wiring and safety must be verified on
the physical robot with wheels raised and a separate power disconnect.
