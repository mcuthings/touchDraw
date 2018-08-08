/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*  Standard C Included Files */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "emwin_support.h"

#include "GUI.h"
#include "GUIDRV_Lin.h"
#include "BUTTON.h"

#include "pin_mux.h"
#include "fsl_sctimer.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define CLEAR_BUTTON_ID (GUI_ID_BUTTON0)

#define COLOR_BUTTONS 8
#define COLOR_BUTTON_FIRST_ID (GUI_ID_USER)
#define COLOR_BUTTON_LAST_ID (COLOR_BUTTON_FIRST_ID + COLOR_BUTTONS - 1)

static GUI_COLOR button_color[COLOR_BUTTONS] = {GUI_WHITE,   GUI_YELLOW, GUI_ORANGE, GUI_RED,
                                                GUI_MAGENTA, GUI_BLUE,   GUI_GREEN,  GUI_BLACK};

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_InitPWM(void)
{
    sctimer_config_t config;
    sctimer_pwm_signal_param_t pwmParam;
    uint32_t event;

    CLOCK_AttachClk(kMAIN_CLK_to_SCT_CLK);

    CLOCK_SetClkDiv(kCLOCK_DivSctClk, 2, true);

    SCTIMER_GetDefaultConfig(&config);

    SCTIMER_Init(SCT0, &config);

    pwmParam.output = kSCTIMER_Out_5;
    pwmParam.level = kSCTIMER_HighTrue;
    pwmParam.dutyCyclePercent = 5;

    SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, 1000U, CLOCK_GetFreq(kCLOCK_Sct), &event);
}


static void cbBackgroundWin(WM_MESSAGE *pMsg)
{
    int widget_id;

    switch (pMsg->MsgId)
    {
        case WM_NOTIFY_PARENT:
            widget_id = WM_GetId(pMsg->hWinSrc);
            if (widget_id >= COLOR_BUTTON_FIRST_ID && widget_id <= COLOR_BUTTON_LAST_ID)
            {
                GUI_SetColor(button_color[widget_id - COLOR_BUTTON_FIRST_ID]);
            }
            else if (widget_id == CLEAR_BUTTON_ID && pMsg->Data.v == WM_NOTIFICATION_CLICKED)
            {
                GUI_Clear();
            }
            break;
        default:
            WM_DefaultProc(pMsg);
    }
}

static void cbCanvasWin(WM_MESSAGE *pMsg)
{
    GUI_PID_STATE *pid_state;
    static GUI_PID_STATE pid_state_0;

    switch (pMsg->MsgId)
    {
        case WM_TOUCH:
            pid_state = (GUI_PID_STATE *)pMsg->Data.p;
            if (pid_state->Pressed)
            {
                if (pid_state_0.Pressed)
                {
                    GUI_DrawLine(pid_state_0.x, pid_state_0.y, pid_state->x, pid_state->y);
                }
                else
                {
                    GUI_DrawPoint(pid_state->x, pid_state->y);
                }
            }
            pid_state_0 = *pid_state;
            break;
        default:
            WM_DefaultProc(pMsg);
    }
}

int main(void)
{
    int i;

    /* Board pin, clock, debug console init */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* Route Main clock to LCD. */
    CLOCK_AttachClk(kMAIN_CLK_to_LCD_CLK);

    /* attach 12 MHz clock to FLEXCOMM2 (I2C master for touch controller) */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

    CLOCK_EnableClock(kCLOCK_Gpio2);

    CLOCK_SetClkDiv(kCLOCK_DivLcdClk, 1, true);

    BOARD_InitPins();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();
    BOARD_InitSDRAM();

    /* Set the back light PWM. */
    BOARD_InitPWM();

    /* emWin start */
    GUI_Init();

    /* Set size and default color for the background window */
    WM_SetSize(WM_HBKWIN, LCD_WIDTH, LCD_HEIGHT);
    WM_SetDesktopColor(GUI_WHITE);

    /* Set callback for the backgroung window */
    WM_SetCallback(WM_HBKWIN, cbBackgroundWin);

    /* Solid color display */
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();

    BUTTON_SetReactOnLevel();

    /* Create the 'clear' button */
    BUTTON_Handle hButtonClear;
    hButtonClear = BUTTON_CreateEx(4, 2, 30, 25, 0, WM_CF_SHOW, 0, CLEAR_BUTTON_ID);
    BUTTON_SetText(hButtonClear, "CLR");

    /* Create color selection buttons */
    BUTTON_Handle hButtonColor[COLOR_BUTTONS];
    for (i = 0; i < COLOR_BUTTONS; i++)
    {
        hButtonColor[i] = BUTTON_CreateEx(4, i * 30 + 32, 30, 25, 0, WM_CF_SHOW, 0, COLOR_BUTTON_FIRST_ID + i);
        BUTTON_SetSkinClassic(hButtonColor[i]);
        BUTTON_SetBkColor(hButtonColor[i], BUTTON_CI_UNPRESSED, button_color[i]);
    }

    /* Create canvas */
    WM_HWIN hCanvas;
    hCanvas = WM_CreateWindowAsChild(35, 0, WM_GetWindowSizeX(WM_HBKWIN) - 35, WM_GetWindowSizeY(WM_HBKWIN), 0,
                                     WM_CF_SHOW, cbCanvasWin, 0);
    /* Select canvas window and leave it selected forever */
    WM_SelectWindow(hCanvas);
    GUI_SetColor(GUI_BLACK);
    GUI_SetDrawMode(GUI_DM_NORMAL);
    GUI_SetPenSize(4);
    GUI_DispStringHCenterAt("Touch and drag to draw.\nClick the CLR button to clear the canvas.",
                            WM_GetWindowSizeX(hCanvas) / 2, WM_GetWindowSizeY(hCanvas) / 2);

    WM_Exec();

    while (1)
    {
        /* Poll touch controller for update */
        if (BOARD_Touch_Poll())
        {
            GUI_MULTIBUF_Begin();
            WM_Exec();
            GUI_MULTIBUF_End();
        }
    }
}
