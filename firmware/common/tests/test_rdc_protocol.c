#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "rdc_protocol.h"

static void test_move_line(void)
{
    char line[RDC_PROTOCOL_MAX_FRAME];
    RdcMessage decoded = {0};
    const size_t length = rdc_encode_move(line, sizeof(line), 50, -25);
    assert(length == strlen("M+0.50/-0.25\n"));
    assert(strcmp(line, "M+0.50/-0.25\n") == 0);
    assert(rdc_parse_line(line, length, &decoded));
    assert(decoded.type == RDC_MESSAGE_MOVE);
    assert(decoded.ratio_x_centi == 50);
    assert(decoded.ratio_y_centi == -25);
}

static void test_rotation_and_mechanism(void)
{
    char line[RDC_PROTOCOL_MAX_FRAME];
    RdcMessage decoded = {0};
    size_t length = rdc_encode_rotation(line, sizeof(line), -100);
    assert(strcmp(line, "R-1.00\n") == 0);
    assert(rdc_parse_line(line, length, &decoded));
    assert(decoded.type == RDC_MESSAGE_ROTATE);
    assert(decoded.ratio_z_centi == -100);

    length = rdc_encode_mechanism(line, sizeof(line), RDC_MESSAGE_MECHANISM_OPEN);
    assert(strcmp(line, "GO\n") == 0);
    assert(rdc_parse_line(line, length, &decoded));
    assert(decoded.type == RDC_MESSAGE_MECHANISM_OPEN);

    length = rdc_encode_mechanism(line, sizeof(line), RDC_MESSAGE_MECHANISM_CLOSE);
    assert(strcmp(line, "GC\n") == 0);
    assert(rdc_parse_line(line, length, &decoded));
    assert(decoded.type == RDC_MESSAGE_MECHANISM_CLOSE);

    length = rdc_encode_mechanism(line, sizeof(line), RDC_MESSAGE_MECHANISM_STOP);
    assert(strcmp(line, "GS\n") == 0);
    assert(rdc_parse_line(line, length, &decoded));
    assert(decoded.type == RDC_MESSAGE_MECHANISM_STOP);
}

static void test_heading_lines(void)
{
    char line[RDC_PROTOCOL_MAX_FRAME];
    RdcMessage decoded = {0};
    const RdcMessageType types[] = {RDC_MESSAGE_HEADING_OFF, RDC_MESSAGE_HEADING_ON, RDC_MESSAGE_HEADING_ZERO};
    const char *expected[] = {"H0\n", "H1\n", "HZ\n"};
    for (unsigned i = 0; i < 3U; ++i) {
        size_t length = rdc_encode_heading(line, sizeof(line), types[i]);
        assert(length == 3U && strcmp(line, expected[i]) == 0);
        assert(rdc_parse_line(line, length, &decoded) && decoded.type == types[i]);
        assert(rdc_encode_heading(line, 3U, types[i]) == 0U);
    }
    assert(!rdc_parse_line("H2\n", 3U, &decoded));
    assert(!rdc_parse_line("HZ\r\n", 4U, &decoded));
    assert(rdc_encode_heading(line, sizeof(line), RDC_MESSAGE_ACK) == 0U);
}

static void test_turn_lines(void)
{
    char line[RDC_PROTOCOL_MAX_FRAME];
    RdcMessage decoded = {0};
    const RdcMessageType types[] = {RDC_MESSAGE_TURN_IDLE, RDC_MESSAGE_TURN_LEFT,
        RDC_MESSAGE_TURN_RIGHT, RDC_MESSAGE_TURN_HALF, RDC_MESSAGE_TURN_CANCEL};
    const char *expected[] = {"Y0\n", "YL\n", "YR\n", "YU\n", "YX\n"};
    for (unsigned i = 0; i < 5U; ++i) {
        size_t length = rdc_encode_turn(line, sizeof(line), types[i]);
        assert(length == 3U && strcmp(line, expected[i]) == 0);
        assert(rdc_parse_line(line, length, &decoded) && decoded.type == types[i]);
        assert(rdc_encode_turn(line, 3U, types[i]) == 0U);
    }
    assert(!rdc_parse_line("YZ\n", 3U, &decoded));
    assert(rdc_encode_turn(line, sizeof(line), RDC_MESSAGE_ACK) == 0U);
}

static void test_ack_line(void)
{
    char line[RDC_PROTOCOL_MAX_FRAME];
    RdcMessage decoded = {0};
    const size_t length = rdc_encode_ack(line, sizeof(line), 42U, 0U);
    assert(strcmp(line, "@A,42,00*47\n") == 0);
    assert(rdc_parse_line(line, length, &decoded));
    assert(decoded.type == RDC_MESSAGE_ACK);
    assert(decoded.sequence == 42U);
    assert(decoded.status == 0U);

    line[4] ^= 1;
    assert(!rdc_parse_line(line, length, &decoded));
}

static void test_reject_malformed_and_crlf(void)
{
    char line[RDC_PROTOCOL_MAX_FRAME];
    RdcMessage decoded = {0};
    size_t length = rdc_encode_move(line, sizeof(line), 0, 0);
    assert(length > 0U);
    assert(!rdc_parse_line(line, length - 1U, &decoded));

    line[length - 1U] = '\r';
    line[length] = '\n';
    assert(!rdc_parse_line(line, length + 1U, &decoded));
    assert(!rdc_parse_line("M+1.01/+0.00\n", 13U, &decoded));
    assert(!rdc_parse_line("R+2.00\n", 7U, &decoded));
    assert(!rdc_parse_line("TX\n", 3U, &decoded));
    assert(!rdc_parse_line("TP\n", 3U, &decoded));
    assert(!rdc_parse_line("TR\n", 3U, &decoded));

    assert(rdc_encode_move(line, sizeof(line), 101, 0) == 0U);
    assert(rdc_encode_rotation(line, sizeof(line), -101) == 0U);
    assert(rdc_encode_mechanism(line, 3U, RDC_MESSAGE_MECHANISM_OPEN) == 0U);
    assert(rdc_encode_mechanism(line, sizeof(line), RDC_MESSAGE_ACK) == 0U);
}

int main(void)
{
    test_move_line();
    test_rotation_and_mechanism();
    test_heading_lines();
    test_turn_lines();
    test_ack_line();
    test_reject_malformed_and_crlf();
    puts("RDC protocol tests passed");
    return 0;
}
