#include "rdc_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t rdc_checksum(const char *data, size_t length)
{
    uint8_t checksum = 0U;
    for (size_t i = 0U; i < length; ++i) {
        checksum ^= (uint8_t)data[i];
    }
    return checksum;
}

static bool ratio_valid(int16_t ratio)
{
    return (ratio >= -RDC_RATIO_LIMIT_CENTI) &&
           (ratio <= RDC_RATIO_LIMIT_CENTI);
}

static unsigned ratio_magnitude(int16_t ratio)
{
    return (unsigned)((ratio < 0) ? -ratio : ratio);
}

size_t rdc_encode_move(char *out, size_t capacity, int16_t x_centi,
                       int16_t y_centi)
{
    if ((out == NULL) || !ratio_valid(x_centi) || !ratio_valid(y_centi)) {
        return 0U;
    }
    const unsigned x = ratio_magnitude(x_centi);
    const unsigned y = ratio_magnitude(y_centi);
    const int n = snprintf(out, capacity, "M%c%u.%02u/%c%u.%02u\n",
                           (x_centi < 0) ? '-' : '+', x / 100U, x % 100U,
                           (y_centi < 0) ? '-' : '+', y / 100U, y % 100U);
    return (n < 0 || (size_t)n >= capacity) ? 0U : (size_t)n;
}

size_t rdc_encode_rotation(char *out, size_t capacity, int16_t z_centi)
{
    if ((out == NULL) || !ratio_valid(z_centi)) {
        return 0U;
    }
    const unsigned z = ratio_magnitude(z_centi);
    const int n = snprintf(out, capacity, "R%c%u.%02u\n",
                           (z_centi < 0) ? '-' : '+', z / 100U, z % 100U);
    return (n < 0 || (size_t)n >= capacity) ? 0U : (size_t)n;
}

size_t rdc_encode_mechanism(char *out, size_t capacity, RdcMessageType command)
{
    if ((out == NULL) || (capacity < 4U)) return 0U;
    const char *line = NULL;
    switch (command) {
        case RDC_MESSAGE_MECHANISM_OPEN:  line = "GO\n"; break;
        case RDC_MESSAGE_MECHANISM_CLOSE: line = "GC\n"; break;
        case RDC_MESSAGE_MECHANISM_STOP:  line = "GS\n"; break;
        default: return 0U;
    }
    const size_t length = strlen(line);
    memcpy(out, line, length + 1U);
    return length;
}

size_t rdc_encode_heading(char *out, size_t capacity, RdcMessageType command)
{
    if ((out == NULL) || (capacity < 4U)) return 0U;
    const char *line;
    switch (command) {
        case RDC_MESSAGE_HEADING_OFF: line = "H0\n"; break;
        case RDC_MESSAGE_HEADING_ON: line = "H1\n"; break;
        case RDC_MESSAGE_HEADING_ZERO: line = "HZ\n"; break;
        default: return 0U;
    }
    memcpy(out, line, 4U);
    return 3U;
}

size_t rdc_encode_turn(char *out, size_t capacity, RdcMessageType command)
{
    if ((out == NULL) || (capacity < 4U)) return 0U;
    const char *line;
    switch (command) {
        case RDC_MESSAGE_TURN_IDLE:   line = "Y0\n"; break;
        case RDC_MESSAGE_TURN_LEFT:   line = "YL\n"; break;
        case RDC_MESSAGE_TURN_RIGHT:  line = "YR\n"; break;
        case RDC_MESSAGE_TURN_HALF:   line = "YU\n"; break;
        case RDC_MESSAGE_TURN_CANCEL: line = "YX\n"; break;
        default: return 0U;
    }
    memcpy(out, line, 4U);
    return 3U;
}

size_t rdc_encode_ack(char *out, size_t capacity, uint16_t sequence,
                      uint8_t status)
{
    if ((out == NULL) || (capacity == 0U)) {
        return 0U;
    }
    char payload[24];
    const int n = snprintf(payload, sizeof(payload), "A,%u,%02X",
                           (unsigned)sequence, (unsigned)status);
    if ((n < 0) || ((size_t)n >= sizeof(payload))) {
        return 0U;
    }
    const uint8_t checksum = rdc_checksum(payload, (size_t)n);
    const int total = snprintf(out, capacity, "@%s*%02X\n", payload,
                               (unsigned)checksum);
    return (total < 0 || (size_t)total >= capacity) ? 0U : (size_t)total;
}

static bool parse_digits(const char *p, size_t count, unsigned *number)
{
    unsigned result = 0U;
    for (size_t i = 0U; i < count; ++i) {
        if ((p[i] < '0') || (p[i] > '9')) {
            return false;
        }
        result = (result * 10U) + (unsigned)(p[i] - '0');
    }
    *number = result;
    return true;
}

static bool parse_ratio(const char *p, int16_t *ratio)
{
    if ((p[0] != '+') && (p[0] != '-')) {
        return false;
    }
    if ((p[1] < '0') || (p[1] > '1') || (p[2] != '.') ||
        (p[3] < '0') || (p[3] > '9') || (p[4] < '0') || (p[4] > '9')) {
        return false;
    }
    unsigned whole = 0U;
    unsigned fraction = 0U;
    if (!parse_digits(p + 1, 1U, &whole) ||
        !parse_digits(p + 3, 2U, &fraction)) {
        return false;
    }
    const unsigned magnitude = whole * 100U + fraction;
    if (magnitude > 100U) {
        return false;
    }
    const int16_t value = (int16_t)magnitude;
    *ratio = (p[0] == '-') ? (int16_t)-value : value;
    return true;
}

static bool parse_ack(const char *line, size_t length, RdcMessage *message)
{
    if ((length < 10U) || (line[0] != '@') ||
        (line[length - 1U] != '\n') ||
        ((length > 1U) && (line[length - 2U] == '\r'))) {
        return false;
    }

    const size_t end = length - 1U;
    size_t star = 0U;
    for (size_t i = 1U; i < end; ++i) {
        if (line[i] == '*') {
            if (star != 0U) {
                return false;
            }
            star = i;
        }
    }
    if ((star == 0U) || (star + 3U != end)) {
        return false;
    }

    char hex[3] = { line[star + 1U], line[star + 2U], '\0' };
    char *hex_end = NULL;
    const unsigned long checksum = strtoul(hex, &hex_end, 16);
    if ((hex_end == hex) || (*hex_end != '\0') || (checksum > 255UL) ||
        (rdc_checksum(line + 1, star - 1U) != (uint8_t)checksum)) {
        return false;
    }

    char payload[24];
    const size_t payload_length = star - 1U;
    if ((payload_length == 0U) || (payload_length >= sizeof(payload))) {
        return false;
    }
    memcpy(payload, line + 1, payload_length);
    payload[payload_length] = '\0';

    char *first = strchr(payload, ',');
    char *second = (first == NULL) ? NULL : strchr(first + 1, ',');
    if ((first == NULL) || (second == NULL) || (strchr(second + 1, ',') != NULL) ||
        (payload[0] != 'A')) {
        return false;
    }
    *first = '\0';
    *second = '\0';
    if (strcmp(payload, "A") != 0) {
        return false;
    }

    char *sequence_end = NULL;
    const unsigned long sequence = strtoul(first + 1, &sequence_end, 10);
    char *status_end = NULL;
    const unsigned long status = strtoul(second + 1, &status_end, 16);
    if ((sequence_end == first + 1) || (*sequence_end != '\0') ||
        (sequence > 65535UL) || (status_end == second + 1) ||
        (*status_end != '\0') || (status > 255UL)) {
        return false;
    }

    memset(message, 0, sizeof(*message));
    message->type = RDC_MESSAGE_ACK;
    message->sequence = (uint16_t)sequence;
    message->status = (uint8_t)status;
    return true;
}

bool rdc_parse_line(const char *line, size_t length, RdcMessage *message)
{
    if ((line == NULL) || (message == NULL) || (length == 0U) ||
        (length > RDC_PROTOCOL_MAX_FRAME) || (line[length - 1U] != '\n') ||
        ((length > 1U) && (line[length - 2U] == '\r'))) {
        return false;
    }
    if (line[0] == '@') {
        return parse_ack(line, length, message);
    }

    RdcMessage decoded = {0};
    if ((length == 13U) && (line[0] == 'M') && (line[6] == '/')) {
        if (!parse_ratio(line + 1, &decoded.ratio_x_centi) ||
            !parse_ratio(line + 7, &decoded.ratio_y_centi)) {
            return false;
        }
        decoded.type = RDC_MESSAGE_MOVE;
    } else if ((length == 7U) && (line[0] == 'R')) {
        if (!parse_ratio(line + 1, &decoded.ratio_z_centi)) {
            return false;
        }
        decoded.type = RDC_MESSAGE_ROTATE;
    } else if ((length == 3U) && (line[2] == '\n')) {
        if (line[0] == 'G' && line[1] == 'O') {
            decoded.type = RDC_MESSAGE_MECHANISM_OPEN;
        } else if (line[0] == 'G' && line[1] == 'C') {
            decoded.type = RDC_MESSAGE_MECHANISM_CLOSE;
        } else if (line[0] == 'G' && line[1] == 'S') {
            decoded.type = RDC_MESSAGE_MECHANISM_STOP;
        } else if (line[0] == 'H' && line[1] == '0') {
            decoded.type = RDC_MESSAGE_HEADING_OFF;
        } else if (line[0] == 'H' && line[1] == '1') {
            decoded.type = RDC_MESSAGE_HEADING_ON;
        } else if (line[0] == 'H' && line[1] == 'Z') {
            decoded.type = RDC_MESSAGE_HEADING_ZERO;
        } else if (line[0] == 'Y' && line[1] == '0') {
            decoded.type = RDC_MESSAGE_TURN_IDLE;
        } else if (line[0] == 'Y' && line[1] == 'L') {
            decoded.type = RDC_MESSAGE_TURN_LEFT;
        } else if (line[0] == 'Y' && line[1] == 'R') {
            decoded.type = RDC_MESSAGE_TURN_RIGHT;
        } else if (line[0] == 'Y' && line[1] == 'U') {
            decoded.type = RDC_MESSAGE_TURN_HALF;
        } else if (line[0] == 'Y' && line[1] == 'X') {
            decoded.type = RDC_MESSAGE_TURN_CANCEL;
        } else {
            return false;
        }
    } else {
        return false;
    }

    *message = decoded;
    return true;
}
