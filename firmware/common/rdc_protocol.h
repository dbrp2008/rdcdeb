#ifndef RDC_PROTOCOL_H
#define RDC_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RDC_PROTOCOL_MAX_FRAME 64U
#define RDC_RATIO_LIMIT_CENTI 100

typedef enum {
    RDC_MESSAGE_NONE = 0,
    RDC_MESSAGE_MOVE,
    RDC_MESSAGE_ROTATE,
    RDC_MESSAGE_MECHANISM_OPEN,
    RDC_MESSAGE_MECHANISM_CLOSE,
    RDC_MESSAGE_MECHANISM_STOP,
    RDC_MESSAGE_HEADING_OFF,
    RDC_MESSAGE_HEADING_ON,
    RDC_MESSAGE_HEADING_ZERO,
    RDC_MESSAGE_TURN_IDLE,
    RDC_MESSAGE_TURN_LEFT,
    RDC_MESSAGE_TURN_RIGHT,
    RDC_MESSAGE_TURN_HALF,
    RDC_MESSAGE_TURN_CANCEL,
    RDC_MESSAGE_ACK
} RdcMessageType;

typedef struct {
    RdcMessageType type;
    uint16_t sequence; /* ACK sequence only; command lines have no sequence. */
    int16_t ratio_x_centi;
    int16_t ratio_y_centi;
    int16_t ratio_z_centi;
    uint8_t status;
} RdcMessage;

/* Controller command lines; ratios are signed hundredths in [-1.00, +1.00]. */
size_t rdc_encode_move(char *out, size_t capacity, int16_t x_centi,
                       int16_t y_centi);
size_t rdc_encode_rotation(char *out, size_t capacity, int16_t z_centi);
/* Mechanism deadman commands: GO=open, GC=close, GS=stop (all LF-terminated). */
size_t rdc_encode_mechanism(char *out, size_t capacity, RdcMessageType command);

/* Heading mode: H0=off, H1=on, HZ=zero reference. */
size_t rdc_encode_heading(char *out, size_t capacity, RdcMessageType command);

/* Turn commands: Y0=idle, YL/YR=90 deg, YU=180 deg, YX=cancel. */
size_t rdc_encode_turn(char *out, size_t capacity, RdcMessageType command);

/* Retained debug ACK line: @A,<seq>,<status-hex>*<xor-hex>\n. */
size_t rdc_encode_ack(char *out, size_t capacity, uint16_t sequence,
                      uint8_t status);
/* Parses one LF-terminated command line or ACK line; CRLF is rejected. */
bool rdc_parse_line(const char *line, size_t length, RdcMessage *message);
uint8_t rdc_checksum(const char *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* RDC_PROTOCOL_H */
