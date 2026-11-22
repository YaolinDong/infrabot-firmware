/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "app_info.h"
#include <assert.h>
#include <stdio.h>

void info_fixture_init(void);

int main(void)
{
    info_fixture_init();
    InfrabotInfoRequest request = {.magic = INFRABOT_MAGIC_INFO_REQUEST, .version = INFRABOT_INFO_V1,
                                   .sequence = 0x78563412U};
    InfrabotInfoReplyV2 reply;
    request.crc32 = infrabot_crc32(&request, offsetof(InfrabotInfoRequest, crc32));
    assert(AppInfo_Reply(&request, &reply) == 154);
    /* Captured from e9a20f5 AppEthernet_SendInfo, with info_fixture_init(). */
    FILE *golden = fopen("tests/data/info_v1.hex", "r");
    assert(golden);
    for (size_t i = 0; i < sizeof(reply.info); ++i) {
        unsigned byte;
        assert(fscanf(golden, "%x", &byte) == 1 && byte == ((const uint8_t *)&reply)[i]);
    }
    unsigned extra;
    assert(fscanf(golden, "%x", &extra) == EOF);
    fclose(golden);

    InfrabotInfoReply expected = reply.info;
    expected.version = request.version = INFRABOT_INFO_V2;
    expected.crc32 = infrabot_crc32(&expected, offsetof(InfrabotInfoReply, crc32));
    request.crc32 = infrabot_crc32(&request, offsetof(InfrabotInfoRequest, crc32));
    assert(AppInfo_Reply(&request, &reply) == sizeof(reply));
    assert(!memcmp(&expected, &reply.info, sizeof(expected)));
    assert(reply.protocol_major == 1 && reply.protocol_minor == 0);
    assert(!strcmp(reply.board, "fire-h743-v2") && !strcmp(reply.arm_model, "H7C bare v1"));
    assert(!strcmp(reply.firmware_version, "01234567890123456789012"));
    assert(reply.crc32 == infrabot_crc32(&reply, offsetof(InfrabotInfoReplyV2, crc32)));
    ++request.crc32;
    assert(!AppInfo_Reply(&request, &reply));
    request.version = 3;
    request.crc32 = infrabot_crc32(&request, offsetof(InfrabotInfoRequest, crc32));
    assert(!AppInfo_Reply(&request, &reply));
    request.version = 1; request.magic = 0;
    request.crc32 = infrabot_crc32(&request, offsetof(InfrabotInfoRequest, crc32));
    assert(!AppInfo_Reply(&request, &reply));
    puts("DMIF: 154-byte V1 golden and extended V2 fields, CRC and rejection checks passed");
}
