#include "state.h"
#include <math.h>
#include "Attitude.h"
#include "ROBOT.h"
#include "FKIK.h"



GaitMode_t GaitMode;
MainState_t Mainstate;
static uint8_t StateMachine_Init(void);
static uint8_t MainState_Update(void);
static uint8_t GaitMode_Update(void);

GaitMode_t GetGaitMode(void)
{
    return GaitMode;
}

MainState_t GetMainState(void)
{
    return Mainstate;
}






