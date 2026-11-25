# RDC firmware workspace

Board applications, shared firmware, and project documentation are grouped by purpose:

```text
firmware/
├── ar-mainboard/              AR boot IR calibration and F1/F2 drive firmware
├── common/                    Shared RDC protocol and host tests
├── controller/                TR hand-controller firmware
├── controller-calibration/    Standalone joystick calibration firmware
├── mainboard/                 Normal MainBoard communication/status firmware
├── mainboard-motor-tuner/     Independent AR/TR bench tuning firmware
└── tr-mainboard/              TR drive MainBoard; pairs with firmware/controller/

docs/
├── communication.md           Wire protocol, wiring, and host test instructions
└── README.md                  Project map and links
```

Build each firmware project from its own directory. The [AR MainBoard drive build](firmware/ar-mainboard/README.md#build-the-drive-firmware) needs no CMake preset. Its optional [EncoderBench preset](firmware/ar-mainboard/README.md#passive-encoder-bench) reads hand-turned wheels without driving them. Read the project instructions before flashing or enabling motor power. The motor tuner is a separate bench image and does not receive joystick commands from the TR Controller.
