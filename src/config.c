#include "config.h"
extern int8_t mode;

void initializeConfigValues() {
    dynamicConfig.speedFilter = 0.03;
    dynamicConfig.steerFilter = 0.03;
    dynamicConfig.steerCoeff = DEFAULT_STEER_COEFFICIENT;
    dynamicConfig.speedCoeff = SPEED_COEFFICIENT;
    dynamicConfig.maxCurrent = DC_CUR_LIMIT;
    dynamicConfig.overcurrent.P = 600;

    dynamicConfig.maxSpeed = 1000;
    dynamicConfig.turboMinSpeed = 450;
    dynamicConfig.turboMaxWeak = 450;
    if(mode == 1) {
        dynamicConfig.maxSpeed = 450;
        dynamicConfig.turboMinSpeed = 2000; // impossible to reach, effectively disabling turbo
    }
    if(mode == 3) {
        dynamicConfig.speedFilter = 0.05;
        dynamicConfig.steerFilter = 0.2;
    }
}
