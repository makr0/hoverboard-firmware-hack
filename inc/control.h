#pragma once

void Nunchuck_Read();
void Nunchuck_Init();
void poweroff();
void remoteControl_Init();

typedef struct tag_remoteControl_struct{
    int speed;
    int steer;
    int button1;
    int button2;
    uint32_t maxInterval;
    uint32_t lastCommandTick;
} remoteControl_struct;
volatile remoteControl_struct remoteControl;

