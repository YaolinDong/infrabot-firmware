/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Golden payloads and decoded IEEE-754 words captured from the HAL transmit
 * hook and feedback decoder of 9903c9c. No board or transport required. */
#include "dm_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void expect(const uint8_t data[8], const char *expected)
{
    char actual[17];
    for (unsigned i = 0; i < 8; i++) snprintf(actual + 2*i, 3, "%02x", data[i]);
    assert(strcmp(actual, expected) == 0);
}

static uint32_t bits(float x)
{
    uint32_t word;
    memcpy(&word, &x, sizeof(word));
    return word;
}

int main(void)
{
    DM_Motor_t motor = {.id=0x123,.master_id=0x321,.position_max=12.5f,.velocity_max=10,.torque_max=28};
    const struct {float p,v,k,d,t; const char *mit,*position,*speed;} cases[] = {
        {-12.5,-10,0,0,-28,"0000000000000000","000048c1000020c1","000020c100000000"},
        {0,0,0,0,0,"8000800000000800","0000000000000000","0000000000000000"},
        {12.5,10,500,5,28,"ffffffffffffffff","0000484100002041","0000204100000000"},
        {-15,-19,-1,-1,-33,"0000000000000000","000070c1000098c1","000098c100000000"},
        {15,19,501,6,33,"ffffffffffffffff","0000704100009841","0000984100000000"},
        {-1.23,2.34,3.45,4.56,5.67,"73679df01ce9799e","a4709dbf8fc21540","8fc2154000000000"}
    };
    uint8_t b[8];
    for (unsigned i=0;i<sizeof cases/sizeof *cases;i++){
        DM_PackMIT(&motor,cases[i].p,cases[i].v,cases[i].k,cases[i].d,cases[i].t,b);
        expect(b,cases[i].mit);
        uint8_t feedback[]={0x13,b[0],b[1],b[2],(uint8_t)((b[3]&0xf0)|(b[6]&0x0f)),b[7],21,22};
        assert(DM_DecodeFeedback(&motor,0x321,feedback,8));
        uint8_t roundtrip[8];DM_PackMIT(&motor,motor.position,motor.velocity,cases[i].k,cases[i].d,motor.torque,roundtrip);
        assert(!memcmp(roundtrip,b,8));
        DM_PackPosition(cases[i].p,cases[i].v,b); expect(b,cases[i].position);
        DM_PackSpeed(cases[i].v,b); expect(b,cases[i].speed);
    }
    const struct {float p,v,i; const char *frame;} pc[]={
        {-12.5,0,0,"000048c100000000"},{0,0,0,"0000000000000000"},
        {12.5,100,1,"0000484110271027"},{-15,-1,-1,"000070c100000000"},
        {15,101,2,"0000704110271027"},{1.23,2.345,.6789,"a4709d3fea00851a"}
    };
    for(unsigned i=0;i<sizeof pc/sizeof *pc;i++){
        DM_PackPositionCurrent(pc[i].p,pc[i].v,pc[i].i,b);expect(b,pc[i].frame);
    }
    const char *mode[]={"fffffffffffffffb","fffffffffffffffc","fffffffffffffffd","fffffffffffffffe"};
    for(unsigned i=0;i<4;i++){DM_PackMode(0xfb+i,b);expect(b,mode[i]);}
    DM_PackRegister(motor.id,0x33,0x42,NULL,b);expect(b,"2301334200000000");
    DM_PackRegister(motor.id,0x55,0x42,(uint8_t[]){1,0xf0,0xaa,0xff},b);expect(b,"2301554201f0aaff");
    assert(DM_RegisterValue(b)==0xffaaf001U);
    DM_PackRegister(motor.id,0xcc,0,NULL,b);expect(b,"2301cc0000000000");
    DM_PackRegister(motor.id,0xaa,1,NULL,b);expect(b,"2301aa0100000000");
    DM_PackRegister(motor.id,0x55,10,(uint8_t[]){2,0,0,0},b);expect(b,"2301550a02000000");

    const uint8_t frames[][8]={{3,0,0,0,0,0,2,9},{0xf3,255,255,255,255,255,255,255},
                               {0x13,128,0,128,8,0,31,27},{0x83,0x42,0x92,0x65,0xab,0xf1,29,42}};
    const uint32_t decoded[][3]={{0xc1480000,0xc1200000,0xc1e00000},{0x41480000,0x41200000,0x41e00000},
                                 {0x39480000,0x3b201000,0x3be01000},{0xc0bff6f0,0xc003c03c,0x415ce1d0}};
    for(unsigned i=0;i<4;i++){
        assert(DM_DecodeFeedback(&motor,0x321,frames[i],8));
        assert(bits(motor.position)==decoded[i][0] && bits(motor.velocity)==decoded[i][1] && bits(motor.torque)==decoded[i][2]);
        assert(motor.feedback_id==3 && motor.state==(frames[i][0]>>4));
        assert(motor.temperature_mos==frames[i][6] && motor.temperature_coil==frames[i][7] && !memcmp(motor.raw,frames[i],8));
    }
    DM_Motor_t saved=motor;
    assert(!DM_DecodeFeedback(&motor,0x123,frames[0],8));
    assert(!DM_DecodeFeedback(&motor,0x321,frames[0],7));
    assert(!DM_DecodeFeedback(&motor,0x321,(uint8_t[8]){1},8));
    assert(!memcmp(&saved,&motor,sizeof motor));
    puts("Damiao wire vectors, clamping, register layout and feedback round trips passed");
}
