/*
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "grlib.h"
#include "widget.h"
#include "canvas.h"
#include "checkbox.h"
#include "container.h"
#include "pushbutton.h"
#include "radiobutton.h"
#include "slider.h"
#include "utils/ustdlib.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
#include "images.h"
#include "driverlib/timer.h"
#include "includes/common.h"
#include "includes/motor.h"
#include "includes/light_sensor_task.h"
#include "includes/accel_sensor_task.h"
#include "includes/shared_variables.h"
#include "includes/display_task.h"
/*-----------------------------------------------------------*/
#include <stdbool.h>
#define RPM_MIN 0
#define RPM_MAX 2500
extern uint32_t accel_threshold;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
typedef enum
{
    PLOT_LIGHT,
    PLOT_ACCEL,
    PLOT_RPM,
    PLOT_POWER
} PlotType;
#define LIGHT_DATA_BUFFER_SIZE 100
uint32_t g_ui32LightDataBuffer[LIGHT_DATA_BUFFER_SIZE] = {0};
uint32_t g_ui32LightDataIndex = 0;
volatile PlotType g_eCurrentPlot;

#define ACCEL_DATA_BUFFER_SIZE 300
uint32_t g_ui32AccelDataBuffer[ACCEL_DATA_BUFFER_SIZE] = {0};
uint32_t g_ui32AccelDataIndex = 0;



uint32_t g_ui32LightDataCount = 0; 
uint32_t g_ui32AccelDataCount = 0;

extern tCanvasWidget g_sCanvas3;
extern tCanvasWidget g_sCanvas1;
//*****************************************************************************
//
// Gloal variable used to store the frequency of the system clock.
//
//*****************************************************************************
uint32_t g_ui32SysClock;
tContext sContext;
Motor_t Motor;
uint32_t luxValue = 10;


// timer
#define MAX_TIME_LENGTH 9
#define MAX_DATE_LENGTH 6

SemaphoreHandle_t xSemaphoreTimer0 = NULL;
extern SemaphoreHandle_t xEmergencyMutex;
volatile uint8_t timer_hours;
volatile uint8_t timer_minutes;
volatile uint8_t timer_seconds;
char time_string[MAX_TIME_LENGTH];
char date[MAX_DATE_LENGTH];
volatile bool g_bLightPlotEnabled = false;
volatile bool g_bAccelPlotEnabled = false;
//*****************************************************************************
//
// The DMA control structure table.
//
//*****************************************************************************
#ifdef ewarm
#pragma data_alignment = 1024
tDMAControlTable psDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(psDMAControlTable, 1024)
tDMAControlTable psDMAControlTable[64];
#else
tDMAControlTable psDMAControlTable[64] __attribute__((aligned(1024)));
#endif

//*****************************************************************************
//
// Forward declarations for the globals required to define the widgets at
// compile-time.
//
//*****************************************************************************
void OnPrevious(tWidget *psWidget);
void OnNext(tWidget *psWidget);
void OnIntroPaint(tWidget *psWidget, tContext *psContext);
void OnMotorPanelPaint(tWidget *psWidget, tContext *psContext);
void OnCanvasPaint(tWidget *psWidget, tContext *psContext);
void OnCheckChange(tWidget *psWidget, uint32_t bSelected);
void OnButtonPress(tWidget *psWidget);
void OnSliderChange(tWidget *psWidget, int32_t i32Value);
extern tCanvasWidget g_psPanels[];

static void vSensorData(uint32_t *data, int dataSize, PlotType plotType, bool filtered);
/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvDisplayTask(void *pvParameters);

void UpdateTime(void);
void SetStartTime(uint8_t hours, uint8_t minutes, uint8_t seconds, const char *set_date);
void intToTwoDigitString(uint8_t num, char *str);
void UpdateTimeString(uint8_t hours, uint8_t minutes, uint8_t seconds, char *timeStr);

void OnLimitSliderChange(tWidget *psWidget, int32_t i32Value);

void UpdateTime(void)
{
    // update the timer values
    timer_seconds++;
    if (timer_seconds >= 60)
    {
        timer_seconds = 0;
        timer_minutes++;
        if (timer_minutes >= 60)
        {
            timer_minutes = 0;
            timer_hours++;
            if (timer_hours >= 24)
            {
                timer_hours = 0;
            }
        }
    }
}

void intToString(uint8_t value, char *str)
{
    char temp[10];
    int i = 0;
    int j = 0;

    do
    {
        temp[i++] = (value % 10) + '0';
        value /= 10;
    } while (value > 0);
    // reverse the string
    while (i > 0)
    {
        str[j++] = temp[--i];
    }
    str[j] = '\0'; // null-terminate the string
}
//
//      Function to set the starting time
//      HH : MM : SS
void SetStartTime(uint8_t hours, uint8_t minutes, uint8_t seconds, const char *set_date)
{
    strcpy((char *)date, set_date);

    timer_hours = hours;
    timer_minutes = minutes;
    timer_seconds = seconds;
}

//
//
//      Function to convert int into string
void intToTwoDigitString(uint8_t num, char *str)
{
    if (num >= 0 && num <= 99)
    {
        str[0] = '0' + num / 10;
        str[1] = '0' + num % 10;
    }
    else
    {
        str[0] = '0';
        str[1] = '0';
    }
    str[2] = '\0';
}

//
//      Function to update the time string
void UpdateTimeString(uint8_t hour, uint8_t minute, uint8_t second, char *timeStr)
{
    char hourStr[3];   // Buffer for hour string
    char minuteStr[3]; // Buffer for minute string
    char secondStr[3]; // Buffer for second string

    // Convert each integer to a two-digit string
    intToTwoDigitString(hour, hourStr);
    intToTwoDigitString(minute, minuteStr);
    intToTwoDigitString(second, secondStr);
    // Concatenate the strings to form the time string
    timeStr[0] = hourStr[0];
    timeStr[1] = hourStr[1];
    timeStr[2] = ':';
    timeStr[3] = minuteStr[0];
    timeStr[4] = minuteStr[1];
    timeStr[5] = ':';
    timeStr[6] = secondStr[0];
    timeStr[7] = secondStr[1];
    timeStr[8] = '\0'; // Null terminator
}

//*****************************************************************************
// RPM Slider on Dashboard panel
//*****************************************************************************
static void
OnRpmChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcText[5];

    // 1) Apply to your motor data
    Motor.desired_rpm = i32Value;

    // 2) Update the slider label
    usprintf(pcText, "%3d", i32Value);
    SliderTextSet((tSliderWidget *)psWidget, pcText);

    // 3) Repaint just that widget
    WidgetPaint(psWidget);
}

// When the user drags the RPM slider, update Motor.desiredRPM
//*****************************************************************************
//
// The second panel, which contains introductory text explaining the
// application.
//
//*****************************************************************************
// Define sliders for motor limits

//*****************************************************************************
//
// The first panel, which demonstrates the graphics primitives.
//
//*****************************************************************************

tSliderWidget g_psSliders[] =
    {
        SliderStruct(g_psPanels + 1, 0, 0,                                       // parent, next, prev
                     &g_sKentec320x240x16_SSD2119, 150, 60, 140, 30, 0, 100, 25, // x, y, width, height
                     (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                      SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                     ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                     &g_sFontCm20, "25%", 0, 0, OnSliderChange),
};

tSliderWidget g_psLimitSliders[] = {
    // Current Lower Limit
    SliderStruct(g_psPanels + 1, &g_psLimitSliders[1], 0, &g_sKentec320x240x16_SSD2119,
                 20, 40, 280, 25, 0, 100, 10, // x, y, width, height
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "10 A", 0, 0, OnLimitSliderChange),
    // Current Upper Limit
    SliderStruct(g_psPanels + 1, &g_psLimitSliders[2], 0, &g_sKentec320x240x16_SSD2119,
                 20, 66, 280, 25, 0, 100, 50,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "50 A", 0, 0, OnLimitSliderChange),
    // Acceleration Lower Limit
    SliderStruct(g_psPanels + 1, &g_psLimitSliders[3], 0, &g_sKentec320x240x16_SSD2119,
                 20, 110, 280, 25, 0, 20, 5,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "5 Rpm/s", 0, 0, OnLimitSliderChange),
    // Acceleration Upper Limit
    SliderStruct(g_psPanels + 1, &g_psLimitSliders[4], 0, &g_sKentec320x240x16_SSD2119,
                 20, 136, 280, 25, 0, 20, 15,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "15 Rpm/s", 0, 0, OnLimitSliderChange),
    // Acceleration Threshold 
    SliderStruct(g_psPanels + 1, 0, 0, &g_sKentec320x240x16_SSD2119,
                 20, 165, 280, 25, 0, 100, 10, // y=170, adjust as needed
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE | SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm20, "Accel Threshold", 0, 0, OnLimitSliderChange),
};

tCanvasWidget g_sLimitSlidersCanvas = CanvasStruct(
    g_psPanels + 1,       // parent
    0,                    // next
    &g_psLimitSliders[0], // child: first slider
    &g_sKentec320x240x16_SSD2119,
    0, 0,                                       // x, y
    320, 190,                                   // width, height
    CANVAS_STYLE_FILL | CANVAS_STYLE_APP_DRAWN, // style
    ClrBlack,                                   // fill color
    0,                                          // outline color
    0,                                          // text color
    &g_sFontCm20,                               // font
    0,                                          // text
    0,                                          // image
    OnMotorPanelPaint                           // paint callback
);

void OnLimitSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcText[8];

    if (psWidget == (tWidget *)&g_psLimitSliders[0])
    {
        Motor.current_limit.lower = i32Value;
        usprintf(pcText, "Min: %d A", i32Value);
        SliderTextSet(&g_psLimitSliders[0], pcText);
    }
    else if (psWidget == (tWidget *)&g_psLimitSliders[1])
    {
        Motor.current_limit.upper = i32Value;
        usprintf(pcText, "Max: %d A", i32Value);
        SliderTextSet(&g_psLimitSliders[1], pcText);
    }
    else if (psWidget == (tWidget *)&g_psLimitSliders[2])
    {
        Motor.acceleration_limit.lower = i32Value;
        usprintf(pcText, "Min: %d Rpm/s", i32Value);
        SliderTextSet(&g_psLimitSliders[2], pcText);
    }
    else if (psWidget == (tWidget *)&g_psLimitSliders[3])
    {
        Motor.acceleration_limit.upper = i32Value;
        usprintf(pcText, "Max: %d Rpm/s", i32Value);
        SliderTextSet(&g_psLimitSliders[3], pcText);
    }
    else if (psWidget == (tWidget *)&g_psLimitSliders[4])
    {   
        xSemaphoreTake(xEmergencyMutex, pdMS_TO_TICKS(100));
        accel_threshold = i32Value;
        xSemaphoreGive(xEmergencyMutex);
        usprintf(pcText, "Threshold: %d", i32Value);
        SliderTextSet(&g_psLimitSliders[4], pcText);
    }
    WidgetPaint(psWidget);
}
#define SLIDER_TEXT_VAL_INDEX 0
#define SLIDER_LOCKED_INDEX 2
#define SLIDER_CANVAS_VAL_INDEX 4

#define NUM_SLIDERS (sizeof(g_psSliders) / sizeof(g_psSliders[0]))

tCanvasWidget g_psCheckBoxIndicators[] =
    {
        CanvasStruct(g_psPanels + 1, g_psSliders, 0,
                     &g_sKentec320x240x16_SSD2119, 230, 134, 50, 42,
                     CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0)};
tCheckBoxWidget g_psCheckBoxes[] =
    {
        CheckBoxStruct(g_psPanels + 1, g_psCheckBoxIndicators, 0,
                       &g_sKentec320x240x16_SSD2119, 40, 134, 189, 42,
                       CB_STYLE_OUTLINE | CB_STYLE_TEXT, 16,
                       0, ClrGray, ClrGreen, &g_sFontCm20, "Select",
                       0, OnCheckChange),
};
#define NUM_CHECK_BOXES (sizeof(g_psCheckBoxes) / \
                         sizeof(g_psCheckBoxes[0]))

// Define push buttons
tCanvasWidget g_psPushButtonIndicators[] =
    {
        CanvasStruct(g_psPanels + 1, g_psPushButtonIndicators + 1, 0,
                     &g_sKentec320x240x16_SSD2119, 40, 85, 20, 20,
                     CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0),
        CanvasStruct(g_psPanels + 1, g_psCheckBoxes, 0,
                     &g_sKentec320x240x16_SSD2119, 90, 85, 20, 20,
                     CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0),
};
tPushButtonWidget g_psPushButtons[] =
    {
        RectangularButtonStruct(g_psPanels + 1, g_psPushButtons + 1, 0,
                                &g_sKentec320x240x16_SSD2119, 30, 35, 40, 40,
                                PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                                ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                                &g_sFontCm22, "1", 0, 0, 0, 0, OnButtonPress),
        CircularButtonStruct(g_psPanels + 1, g_psPushButtonIndicators, 0,
                             &g_sKentec320x240x16_SSD2119, 100, 55, 20,
                             PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                             ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                             &g_sFontCm22, "3", 0, 0, 0, 0, OnButtonPress),
};
tPushButtonWidget g_sEStopButton = RectangularButtonStruct(
    g_psPanels, 0, 0, &g_sKentec320x240x16_SSD2119,
    210, 100, 80, 40, // x, y, width, height (adjust as needed)
    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    ClrGray, ClrGray, ClrWhite, ClrWhite,
    &g_sFontCm20, "E-STOP", 0, 0, 0, 0,
    OnButtonPress);

// Update Stop button position to make space for E-STOP
tPushButtonWidget g_sStopButton = RectangularButtonStruct(
    g_psPanels, &g_sEStopButton, 0, &g_sKentec320x240x16_SSD2119,
    120, 100, 80, 40,
    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    ClrRed, ClrGray, ClrWhite, ClrWhite,
    &g_sFontCm20, "Stop", 0, 0, 0, 0,
    OnButtonPress);

tPushButtonWidget g_sStartButton = RectangularButtonStruct(
    g_psPanels, &g_sStopButton, 0, &g_sKentec320x240x16_SSD2119,
    30, 120 - 20, 80, 40,
    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    ClrGreen, ClrGray, ClrWhite, ClrWhite,
    &g_sFontCm20, "Start", 0, 0, 0, 0,
    OnButtonPress);
tSliderWidget g_psRpmSlider[] = {
    SliderStruct(
        g_psPanels, &g_sStartButton, 0, &g_sKentec320x240x16_SSD2119,
        70, 65 - 15, 200, 30,
        0, 2500, 0,
        (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
         SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
        ClrGray, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
        &g_sFontCm20,
        "0", 0, 0,
        OnRpmChange),
};
#define NUM_RPM_SLIDERS (sizeof(g_psRpmSlider) / sizeof(g_psRpmSlider[0]))
Canvas(g_sDashboard, g_psPanels, g_psRpmSlider, 0, &g_sKentec320x240x16_SSD2119, 0, 0,
       320, 240, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnIntroPaint);

#define NUM_PUSH_BUTTONS (sizeof(g_psPushButtons) / \
                          sizeof(g_psPushButtons[0]))
uint32_t g_ui32ButtonState;

//*****************************************************************************
//
// The third panel, which demonstrates the canvas widget.
//
//*****************************************************************************
// Forward declarations for button handlers
void OnPlotSelectButton(tWidget *psWidget);
// Forward declarations for plot select buttons
extern tPushButtonWidget g_sPlotBtnAccel;
extern tPushButtonWidget g_sPlotBtnRPM;
extern tPushButtonWidget g_sPlotBtnPower;

// Now define the buttons in order
tPushButtonWidget g_sPlotBtnLight = RectangularButtonStruct(
    g_psPanels + 2, &g_sPlotBtnAccel, 0, &g_sKentec320x240x16_SSD2119,
    25, 5, 70, 28, // x, y, width, height
    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    ClrGray, ClrSilver, ClrWhite, ClrBlack,
    &g_sFontCm18, "Light", 0, 0, 0, 0,
    OnPlotSelectButton);

tPushButtonWidget g_sPlotBtnAccel = RectangularButtonStruct(
    g_psPanels + 2, &g_sPlotBtnRPM, 0, &g_sKentec320x240x16_SSD2119,
    105, 5, 90, 28,
    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    ClrGray, ClrSilver, ClrWhite, ClrBlack,
    &g_sFontCm18, "Acceleration", 0, 0, 0, 0,
    OnPlotSelectButton);

tPushButtonWidget g_sPlotBtnRPM = RectangularButtonStruct(
    g_psPanels + 2, &g_sPlotBtnPower, 0, &g_sKentec320x240x16_SSD2119,
    205, 5, 60, 28,
    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    ClrGray, ClrSilver, ClrWhite, ClrBlack,
    &g_sFontCm18, "RPM", 0, 0, 0, 0,
    OnPlotSelectButton);

// Change the last plot button's next pointer to the plot canvas
tPushButtonWidget g_sPlotBtnPower = RectangularButtonStruct(
    g_psPanels + 2, &g_sCanvas3, 0, &g_sKentec320x240x16_SSD2119,
    270, 5, 50, 28,
    PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
    ClrGray, ClrSilver, ClrWhite, ClrBlack,
    &g_sFontCm18, "Power", 0, 0, 0, 0,
    OnPlotSelectButton);

Canvas(g_sCanvas3, g_psPanels + 2, 0, 0,
       &g_sKentec320x240x16_SSD2119, 0, 35, 320, 155,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN, 0, ClrGray,
       0, 0, 0, 0, OnCanvasPaint);

// Now, the sensor panel canvas should have the first plot button as its child
tCanvasWidget g_sSensorPanelCanvas = CanvasStruct(
    g_psPanels + 2,   // parent
    0,                // next
    &g_sPlotBtnLight, // child: first plot select button
    &g_sKentec320x240x16_SSD2119,
    0, 0,              // x, y
    320, 190,          // width, height
    CANVAS_STYLE_FILL, // style
    ClrBlack, 0, 0, 0, 0, 0, 0);

// Implement the button handler
void OnPlotSelectButton(tWidget *psWidget)
{
    // Reset all plot buttons to default color
    PushButtonFillColorSet(&g_sPlotBtnLight, ClrGray);
    PushButtonFillColorSet(&g_sPlotBtnAccel, ClrGray);
    PushButtonFillColorSet(&g_sPlotBtnRPM, ClrGray);
    PushButtonFillColorSet(&g_sPlotBtnPower, ClrGray);

    g_bLightPlotEnabled = false; // Default: plotting disabled
    g_bAccelPlotEnabled = false; // Default: plotting disabled

    if (psWidget == (tWidget *)&g_sPlotBtnLight)
    {
        g_eCurrentPlot = PLOT_LIGHT;
        PushButtonFillColorSet(&g_sPlotBtnLight, ClrYellow);

        // Reset buffer and index for new plot
        for (uint32_t i = 0; i < LIGHT_DATA_BUFFER_SIZE; i++)
            g_ui32LightDataBuffer[i] = 0;
        g_ui32LightDataIndex = 0;
        g_ui32LightDataCount = 0;

        g_bLightPlotEnabled = true; // Enable plotting for light

        // Draw axes and labels immediately
        tRectangle sRect = {10, 40, 310, 180};
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
        GrContextForegroundSet(&sContext, ClrWhite);
        GrLineDraw(&sContext, 10, 180, 310, 180); // X-axis
        GrLineDraw(&sContext, 10, 40, 10, 180);   // Y-axis
        // GrContextFontSet(&sContext, g_psFontFixed6x8);
        // GrStringDraw(&sContext, "Time", -1, 160, 192, false);
        // GrStringDraw(&sContext, "Lux", -1, 20, 35, false);
    }
    else if (psWidget == (tWidget *)&g_sPlotBtnAccel)
    {
        g_eCurrentPlot = PLOT_ACCEL;
        PushButtonFillColorSet(&g_sPlotBtnAccel, ClrYellow);
        
        // Reset buffer and index for new plot
        for (uint32_t i = 0; i < ACCEL_DATA_BUFFER_SIZE; i++)
            g_ui32AccelDataBuffer[i] = 0;
        g_ui32AccelDataIndex = 0;
        g_ui32AccelDataCount = 0;

        g_bAccelPlotEnabled = true; // Enable plotting for light

        // Draw axes and labels immediately
        tRectangle sRect = {10, 40, 310, 180};
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
        GrContextForegroundSet(&sContext, ClrWhite);
        GrLineDraw(&sContext, 10, 180, 310, 180); // X-axis
        GrLineDraw(&sContext, 10, 40, 10, 180);   // Y-axis


        
    }
    else if (psWidget == (tWidget *)&g_sPlotBtnRPM)
    {
        g_eCurrentPlot = PLOT_RPM;
        PushButtonFillColorSet(&g_sPlotBtnRPM, ClrYellow);
        // (reset rpm buffer here if you add it)
    }
    else if (psWidget == (tWidget *)&g_sPlotBtnPower)
    {
        g_eCurrentPlot = PLOT_POWER;
        PushButtonFillColorSet(&g_sPlotBtnPower, ClrYellow);
        // (reset power buffer here if you add it)
    }

    // Repaint all plot buttons to update highlight
    WidgetPaint((tWidget *)&g_sPlotBtnLight);
    WidgetPaint((tWidget *)&g_sPlotBtnAccel);
    WidgetPaint((tWidget *)&g_sPlotBtnRPM);
    WidgetPaint((tWidget *)&g_sPlotBtnPower);

    // Force a repaint of the third panel
    WidgetPaint((tWidget *)&g_psPanels[2]);
}
//*****************************************************************************
//
// An array of canvas widgets, one per panel.  Each canvas is filled with
// black, overwriting the contents of the previous panel.
//
//*****************************************************************************
tCanvasWidget g_psPanels[] =
    {
        // Dashboard panel
        CanvasStruct(0, 0, &g_sDashboard, &g_sKentec320x240x16_SSD2119, 0, 0,
                     320, 190, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
        // Motor Control panel (second panel)
        CanvasStruct(0, 0, &g_sLimitSlidersCanvas, &g_sKentec320x240x16_SSD2119, 0, 0,
                     320, 190, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
        // Sensor Graphs panel
        CanvasStruct(0, 0, &g_sSensorPanelCanvas, &g_sKentec320x240x16_SSD2119, 0, 0, 320,
                     190, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
};

//*****************************************************************************
//
// The number of panels.
//
//*****************************************************************************
#define NUM_PANELS (sizeof(g_psPanels) / sizeof(g_psPanels[0]))

//*****************************************************************************
//
// The names for each of the panels, which is displayed at the bottom of the
// screen.
//
//*****************************************************************************
char *g_pcPanei32Names[] =
    {
        "     Dashboard     ",
        "     Motor Control     ",
        "     Sensor Graphs     ",
};

//*****************************************************************************
//
// The buttons and text across the bottom of the screen.
//
//*****************************************************************************
RectangularButton(g_sPrevious, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 195,
                  40, 40, PB_STYLE_FILL, ClrBlack, ClrBlack, 0, ClrSilver,
                  &g_sFontCm20, "-", g_pui8Blue50x50, g_pui8Blue50x50Press, 0, 0,
                  OnPrevious);

Canvas(g_sTitle, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 50, 200, 220, 30,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE, 0, 0, ClrSilver,
       &g_sFontCm20, 0, 0, 0);

RectangularButton(g_sNext, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 275, 195,
                  40, 40, PB_STYLE_IMG | PB_STYLE_TEXT, ClrBlack, ClrBlack, 0,
                  ClrSilver, &g_sFontCm20, "+", g_pui8Blue50x50,
                  g_pui8Blue50x50Press, 0, 0, OnNext);

//*****************************************************************************
//
// The panel that is currently being displayed.
//
//*****************************************************************************
uint32_t g_ui32Panel;

//*****************************************************************************
//
// Handles presses of the previous panel button.
//
//*****************************************************************************
void OnPrevious(tWidget *psWidget)
{
    //
    // There is nothing to be done if the first panel is already being
    // displayed.
    //
    if (g_ui32Panel == 0)
    {
        return;
    }

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Decrement the panel index.
    //
    g_ui32Panel--;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if this is the first panel.
    //
    if (g_ui32Panel == 0)
    {
        //
        // Clear the previous button from the display since the first panel is
        // being displayed.
        //
        PushButtonImageOff(&g_sPrevious);
        PushButtonTextOff(&g_sPrevious);
        PushButtonFillOn(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if the previous panel was the last panel.
    //
    if (g_ui32Panel == (NUM_PANELS - 2))
    {
        //
        // Display the next button.
        //
        PushButtonImageOn(&g_sNext);
        PushButtonTextOn(&g_sNext);
        PushButtonFillOff(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }
}

//*****************************************************************************
//
// Handles presses of the next panel button.
//
//*****************************************************************************
void OnNext(tWidget *psWidget)
{
    //
    // There is nothing to be done if the last panel is already being
    // displayed.
    //
    if (g_ui32Panel == (NUM_PANELS - 1))
    {
        return;
    }

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Increment the panel index.
    //
    g_ui32Panel++;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if the previous panel was the first panel.
    //
    if (g_ui32Panel == 1)
    {
        //
        // Display the previous button.
        //
        PushButtonImageOn(&g_sPrevious);
        PushButtonTextOn(&g_sPrevious);
        PushButtonFillOff(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if this is the last panel.
    //
    if (g_ui32Panel == (NUM_PANELS - 1))
    {
        //
        // Clear the next button from the display since the last panel is being
        // displayed.
        //
        PushButtonImageOff(&g_sNext);
        PushButtonTextOff(&g_sNext);
        PushButtonFillOn(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }
}

//*****************************************************************************
//
// Handles paint requests for the Dashboard canvas widget.
//
//*****************************************************************************
//-----------------------------------------------------------------------------
// Draw the Dashboard: a little color‐block for MotorState + text
void OnIntroPaint(tWidget *psWidget, tContext *psContext)
{
    tRectangle sRect;
    const char *pcState;
    // Clear the status area first
    sRect.i16XMin = 130;
    sRect.i16YMin = 0 - 15;
    sRect.i16XMax = 220;
    sRect.i16YMax = 60 - 15; // Covers the status and LED area
    GrContextForegroundSet(psContext, ClrBlack);
    GrRectFill(psContext, &sRect);
    // LED
    sRect.i16XMin = 10;
    sRect.i16YMin = 10 - 15;
    sRect.i16XMax = 30;
    sRect.i16YMax = 30 - 15;
    // Dynamically update STOP button label and color

     // Dynamically update STOP and START button label and color
    switch (Motor.MotorState)
    {
    case IDLE:
        PushButtonTextSet(&g_sStopButton, "Stop");
        PushButtonFillColorSet(&g_sStopButton, ClrGray);      // Stop button greyed
        PushButtonFillColorSet(&g_sStartButton, ClrGreen);    // Start button active
        GrContextForegroundSet(psContext, ClrGray);
        pcState = "IDLE";
        break;
    case RUNNING:
        PushButtonFillColorSet(&g_sStartButton, ClrGray);     // Start button greyed
        PushButtonTextSet(&g_sStopButton, "Stop");
        PushButtonFillColorSet(&g_sStopButton, ClrRed);       // Stop button active
        GrContextForegroundSet(psContext, ClrBlue);
        pcState = "RUNNING";
        break;
    case STOP:
        GrContextForegroundSet(psContext, ClrRed);
        PushButtonTextSet(&g_sStopButton, "Stop");
        PushButtonFillColorSet(&g_sStopButton, ClrGray);      // Stop button greyed
        PushButtonFillColorSet(&g_sStartButton, ClrGreen);    // Start button active
        pcState = "STOPPED";
        break;
    case ESTOP:
        PushButtonFillColorSet(&g_sStartButton, ClrGreen);
        PushButtonTextSet(&g_sStopButton, "ACK");
        PushButtonFillColorSet(&g_sStopButton, ClrOrange);
        GrContextForegroundSet(psContext, ClrOrange);
        pcState = "E-STOP";
        break;
    default: // assume ESTOP or fault
        GrContextForegroundSet(psContext, ClrWhite);
        pcState = "E-STOP";
    }
    GrCircleFill(psContext, 240, 40 - 15, 8);
    GrContextForegroundSet(psContext, ClrWhite);
    GrCircleDraw(psContext, 240, 40 - 15, 8);

    // Draw/paint the E-STOP button
    WidgetPaint((tWidget *)&g_sStopButton);
    WidgetPaint((tWidget *)&g_sStartButton);
    WidgetPaint((tWidget *)&g_sEStopButton);
    // E-STOP button state logic
    if (Motor.MotorState == ESTOP)
    {
        PushButtonTextSet(&g_sEStopButton, "ACK");
        PushButtonFillColorSet(&g_sEStopButton, ClrOrange);
        PushButtonFillOn(&g_sEStopButton);
        PushButtonTextOn(&g_sEStopButton);
    }
    else
    {
        PushButtonTextSet(&g_sEStopButton, "E-STOP");
        PushButtonFillColorSet(&g_sEStopButton, ClrGray);
        PushButtonFillOn(&g_sEStopButton);
        PushButtonTextOn(&g_sEStopButton);
    }
    WidgetPaint((tWidget *)&g_sEStopButton);

    // Draw the "MOTOR STATUS:" label
    GrContextFontSet(psContext, &g_sFontCm18);
    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDraw(psContext, "Motor Status:", -1, 20, 35 - 15, false);
    // Draw the mode text below the status
    GrContextFontSet(psContext, &g_sFontCm18);
    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDraw(psContext, pcState, -1, 130, 35 - 15, false);

    // Draw the "RPM:" label
    GrContextFontSet(psContext, &g_sFontCm18);
    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDraw(psContext, "RPM:", -1, 20, 70 - 15, false);

    // Draw day/night indicator in top-right corner
    tRectangle dayNightRect;
    dayNightRect.i16XMin = 320 - 60; // 60px wide, right-aligned
    dayNightRect.i16YMin = 8;
    dayNightRect.i16XMax = 319 - 8; // 8px padding from right
    dayNightRect.i16YMax = 8 + 24;  // 24px tall
    luxValue = 3;
    if (luxValue < 5)
    {
        // Night: blue box, white text "Night"
        GrContextForegroundSet(psContext, ClrBlack);
        GrRectFill(psContext, &dayNightRect);
        // Draw "Night" at bottom left corner, y decreased by 50
        GrContextForegroundSet(psContext, ClrBlueViolet);
        GrContextFontSet(psContext, &g_sFontCm18);
        GrStringDraw(psContext, "Night", -1, 8, 150, false);
    }
    else
    {
        // Day: yellow box, grey text "Day"
        GrContextForegroundSet(psContext, ClrBlack);
        GrRectFill(psContext, &dayNightRect);
        // Draw "Day" at bottom left corner, y decreased by 50
        GrContextForegroundSet(psContext, ClrGoldenrod);
        GrContextFontSet(psContext, &g_sFontCm18);
        GrStringDraw(psContext, "Day", -1, 8, 150, false);
    }
    // display current date below
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDraw(psContext, "27/05/25", -1, 20, 170, false);
}
void OnMotorPanelPaint(tWidget *psWidget, tContext *psContext)
{
    GrContextFontSet(psContext, &g_sFontCm20);
    GrContextForegroundSet(psContext, ClrWhite);
    // draw a Threshold Parameter label
    // Center "Threshold Parameters" at the top of the panel
    GrStringDrawCentered(psContext, "Threshold Parameters", -1, 160, 10, false);
    // Draw labels for each slider
    GrContextFontSet(psContext, &g_sFontCm16);
    GrContextForegroundSet(psContext, ClrGray);
    GrStringDraw(psContext, "Current Upper/Lower", -1, 10, 25, false);
    // Add a border box around the text and sliders
    // tRectangle sRect;
    // sRect.i16XMin = 10;
    // sRect.i16YMin = 10;
    // sRect.i16XMax = 310;
    // sRect.i16YMax = 190;
    // GrContextForegroundSet(psContext, ClrGray);
    // GrRectDraw(psContext, &sRect);
    GrStringDraw(psContext, "Acceleration Upper/Lower", -1, 10, 95, false);
    // GrStringDraw(psContext, "Current Upper", -1, 10, 70, false);
    // GrStringDraw(psContext, "Accel Lower", -1, 10, 110, false);
    // GrStringDraw(psContext, "Accel Upper", -1, 10, 150, false);
}
//*****************************************************************************
//
// Handles paint requests for the canvas demonstration widget.
//
//*****************************************************************************
void OnCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    tRectangle sRect = {10, 40, 310, 180};
    if (g_eCurrentPlot == PLOT_LIGHT && g_ui32LightDataCount == 0)
    {
        // Draw axes and labels only
        
        GrContextForegroundSet(psContext, ClrBlack);
        GrRectFill(psContext, &sRect);
        GrContextForegroundSet(psContext, ClrWhite);
        GrLineDraw(psContext, 10, 175, 310, 175); // X-axis
        GrLineDraw(psContext, 10, 40, 10, 175);   // Y-axis
        // GrContextFontSet(psContext, g_psFontFixed6x8);
        // GrStringDraw(psContext, "Time", -1, 160, 192, false);
        // GrStringDraw(psContext, "Lux", -1, 4, 30, false);
    }
    if (g_eCurrentPlot == PLOT_ACCEL && g_ui32AccelDataCount == 0)
    {
        // Draw axes and labels only
        GrContextForegroundSet(psContext, ClrBlack); // Use black, not pink
        GrRectFill(psContext, &sRect);
        GrContextForegroundSet(psContext, ClrWhite); // Use white for axes
        GrLineDraw(psContext, 10, 175, 310, 175); // X-axis
        GrLineDraw(psContext, 10, 40, 10, 175);   // Y-axis
    }
}

//*****************************************************************************
//
// Handles change notifications for the check box widgets.
//
//*****************************************************************************
void OnCheckChange(tWidget *psWidget, uint32_t bSelected)
{
    uint32_t ui32Idx;

    //
    // Find the index of this check box.
    //
    for (ui32Idx = 0; ui32Idx < NUM_CHECK_BOXES; ui32Idx++)
    {
        if (psWidget == (tWidget *)(g_psCheckBoxes + ui32Idx))
        {
            break;
        }
    }

    //
    // Return if the check box could not be found.
    //
    if (ui32Idx == NUM_CHECK_BOXES)
    {
        return;
    }

    //
    // Set the matching indicator based on the selected state of the check box.
    //
    CanvasImageSet(g_psCheckBoxIndicators + ui32Idx,
                   bSelected ? g_pui8LightOn : g_pui8LightOff);
    WidgetPaint((tWidget *)(g_psCheckBoxIndicators + ui32Idx));
}

//*****************************************************************************
//
// Handles press notifications for the push button widgets.
//
//*****************************************************************************
void OnButtonPress(tWidget *psWidget)
{
    uint32_t ui32Idx;

    // Start button: only works if not in ESTOP
    if (psWidget == (tWidget *)&g_sStartButton)
    {
        if (Motor.MotorState != ESTOP)
        {
            Motor.MotorState = RUNNING;
            WidgetPaint((tWidget *)&g_sDashboard);
        }
        return;
    }

    // Stop button: only stops the motor if running
    if (psWidget == (tWidget *)&g_sStopButton)
    {
        if (Motor.MotorState == RUNNING)
        {
            Motor.MotorState = STOP;
            WidgetPaint((tWidget *)&g_sDashboard);
        }
        return;
    }
    // E-STOP/ACK button logic
    if (psWidget == (tWidget *)&g_sEStopButton)
    {
        if (Motor.MotorState != ESTOP)
        {
            // Trigger E-STOP
            Motor.MotorState = ESTOP;
            // Change button to ACK (orange, enabled)
            PushButtonTextSet(&g_sEStopButton, "ACK");
            PushButtonFillColorSet(&g_sEStopButton, ClrOrange);
            PushButtonFillOn(&g_sEStopButton);
            PushButtonTextOn(&g_sEStopButton);
            WidgetPaint((tWidget *)&g_sEStopButton);
        }
        else
        {
            // ACK pressed, return to STOP and grey out E-STOP
            Motor.MotorState = STOP;
            PushButtonTextSet(&g_sEStopButton, "E-STOP");
            PushButtonFillColorSet(&g_sEStopButton, ClrGray);
            PushButtonFillOn(&g_sEStopButton);
            PushButtonTextOn(&g_sEStopButton);
            WidgetPaint((tWidget *)&g_sEStopButton);
        }
        WidgetPaint((tWidget *)&g_sDashboard);
        return;
    }
    // Handle push buttons on the second panel as before
    for (ui32Idx = 0; ui32Idx < NUM_PUSH_BUTTONS; ui32Idx++)
    {
        if (psWidget == (tWidget *)(g_psPushButtons + ui32Idx))
        {
            break;
        }
    }

    if (ui32Idx == NUM_PUSH_BUTTONS)
    {
        return;
    }

    // Example: toggle indicator for push buttons on second panel
    g_ui32ButtonState ^= 1 << ui32Idx;
    CanvasImageSet(g_psPushButtonIndicators + ui32Idx,
                   (g_ui32ButtonState & (1 << ui32Idx)) ? g_pui8LightOn : g_pui8LightOff);
    WidgetPaint((tWidget *)(g_psPushButtonIndicators + ui32Idx));
}

//*****************************************************************************
//
// Handles notifications from the slider controls.
//
//*****************************************************************************
void OnSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcCanvasText[5];
    static char pcSliderText[5];

    //
    // Is this the widget whose value we mirror in the canvas widget and the
    // locked slider?
    //
    if (psWidget == (tWidget *)&g_psSliders[SLIDER_CANVAS_VAL_INDEX])
    {
        //
        // Yes - update the canvas to show the slider value.
        //
        // usprintf(pcCanvasText, "%3d%%", i32Value);
        // CanvasTextSet(&g_sSliderValueCanvas, pcCanvasText);
        // WidgetPaint((tWidget *)&g_sSliderValueCanvas);

        //
        // Also update the value of the locked slider to reflect this one.
        //
        SliderValueSet(&g_psSliders[SLIDER_LOCKED_INDEX], i32Value);
        WidgetPaint((tWidget *)&g_psSliders[SLIDER_LOCKED_INDEX]);
    }

    if (psWidget == (tWidget *)&g_psSliders[SLIDER_TEXT_VAL_INDEX])
    {
        //
        // Yes - update the canvas to show the slider value.
        //
        usprintf(pcSliderText, "%3d%%", i32Value);
        SliderTextSet(&g_psSliders[SLIDER_TEXT_VAL_INDEX], pcSliderText);
        WidgetPaint((tWidget *)&g_psSliders[SLIDER_TEXT_VAL_INDEX]);
    }
}

void vCreateDisplayTask(void)
{
    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name Hello task - for debug only as it is
     *    not used by the kernel.
     *  - The size of the stack to allocate to the task.
     *  - No parameter passed to the task
     *  - The priority assigned to the task.
     *  - The task handle is NULL */
    xTaskCreate(prvDisplayTask,
                "Display Task",
                1024,
                NULL,
                tskIDLE_PRIORITY,
                NULL);
}
/*-----------------------------------------------------------*/

static void prvDisplayTask(void *pvParameters)
{
    // UARTprintf("Display task started\n");
    tRectangle sRect;

    struct AMessage xRxedStructure;
    uint32_t buffer_data[100] = {0};
    uint32_t data_index = 0;
    bool plotRawData = false; // Flag to toggle between raw and filtered data
    //
    // Add the title block and the previous and next buttons to the widget
    // tree.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sTitle);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sNext);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sPrevious);
    //
    // Add the first panel to the widget tree.
    //
    g_ui32Panel = 0;
    WidgetAdd(WIDGET_ROOT, (tWidget *)g_psPanels);
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[0]);
    // WidgetAdd(WIDGET_ROOT, (tWidget *)&g_psRpmSlider[0]);
    //
    //  Issue the initial paint request to the widgets.
    //
    WidgetPaint(WIDGET_ROOT);

    //
    // Loop forever handling widget messages.
    //
    for (;;)
    {
        // Check for event bits
        EventBits_t uxBits = xEventGroupWaitBits(
            xEventGroup,
            EVENT_HIGH_THRESHOLD | EVENT_LOW_THRESHOLD | EVENT_BTN_TOGGLE,
            pdTRUE,  // Clear bits after reading
            pdFALSE, // Wait for any bit
            0);      // Non-blocking

        if (uxBits & EVENT_HIGH_THRESHOLD)
        {
            UARTprintf("Warning: High threshold exceeded!\n");
        }

        if (uxBits & EVENT_LOW_THRESHOLD)
        {
            //UARTprintf("Warning: Low threshold exceeded!\n");
        }

        if (uxBits & EVENT_BTN_TOGGLE)
        {
            plotRawData = !plotRawData;
            UARTprintf("Toggled plot mode: %s\n", plotRawData ? "Raw Data" : "Filtered Data");
        }

        // block until ISR gives semaphore
        if (xSemaphoreTake(xSemaphoreTimer0, portMAX_DELAY) == pdTRUE)
        {
            //
            // Process any messages in the widget message queue.
            //
            WidgetMessageQueueProcess();

            if (g_ui32Panel == 0)
            {
                GrContextForegroundSet(&sContext, ClrBlack);
                GrContextFontSet(&sContext, &g_sFontCm20);
                GrStringDrawCentered(&sContext, time_string, -1,
                                     120, 177, 0);

                // Draw the current time
                UpdateTimeString(timer_hours, timer_minutes, timer_seconds, time_string);
                GrContextForegroundSet(&sContext, ClrWhite);
                GrContextFontSet(&sContext, &g_sFontCm20);
                GrStringDrawCentered(&sContext, time_string, -1,
                                     120, 177, 0);
            }
           
        }
        if (g_ui32Panel == 2 && g_eCurrentPlot == PLOT_LIGHT && g_bLightPlotEnabled)
        {
            if (xQueueReceive(xLightQueue, &(xRxedStructure), (TickType_t)10) == pdPASS)
            {
                g_ui32LightDataBuffer[g_ui32LightDataIndex] = plotRawData ? xRxedStructure.uRaw : xRxedStructure.uFiltered;
                g_ui32LightDataIndex = (g_ui32LightDataIndex + 1) % LIGHT_DATA_BUFFER_SIZE;
                if (g_ui32LightDataCount < LIGHT_DATA_BUFFER_SIZE)
                    g_ui32LightDataCount++;
                // print the buffer recieved data
                //uint32_t prevIndex = (g_ui32LightDataIndex == 0) ? (LIGHT_DATA_BUFFER_SIZE - 1) : (g_ui32LightDataIndex - 1);
                //UARTprintf("Light Data: %d, Count: %d\n", g_ui32LightDataBuffer[prevIndex], g_ui32LightDataCount); 

                vSensorData(g_ui32LightDataBuffer, g_ui32LightDataCount, PLOT_LIGHT, plotRawData);
            }
            else{
                 //UARTprintf("No Light Data received\n");
            }
        }
        if (g_ui32Panel == 2 && g_eCurrentPlot == PLOT_ACCEL && g_bAccelPlotEnabled){
            if (xQueueReceive(xAccelQueue, &xRxedStructure, (TickType_t)10) == pdPASS)
            {
                g_ui32AccelDataBuffer[g_ui32AccelDataIndex] = plotRawData ? xRxedStructure.uRaw : xRxedStructure.uFiltered;
                g_ui32AccelDataIndex = (g_ui32AccelDataIndex + 1) % ACCEL_DATA_BUFFER_SIZE;
                //UARTprintf("Accel Data: %d, Count: %d\n", g_ui32AccelDataBuffer[g_ui32AccelDataIndex], g_ui32AccelDataCount);
                uint32_t prevIndex = (g_ui32AccelDataIndex == 0) ? (ACCEL_DATA_BUFFER_SIZE - 1) : (g_ui32AccelDataIndex - 1);
                //UARTprintf("%d\n", plotRawData ? xRxedStructure.uRaw : xRxedStructure.uFiltered);
               
                if (g_ui32AccelDataCount < ACCEL_DATA_BUFFER_SIZE)
                    g_ui32AccelDataCount++;
                vSensorData(g_ui32AccelDataBuffer, g_ui32AccelDataCount, PLOT_ACCEL, plotRawData);
            }
            else{
                //UARTprintf("Accel Data2: %d\n", g_ui32AccelDataBuffer[g_ui32AccelDataIndex]);
            }
        }
    }
}
static void vSensorData(uint32_t *data, int dataSize, PlotType plotType, bool filtered)
{
    // Plot area
    uint32_t xStart = 10;  // left edge of plot
    uint32_t yStart = 175; // bottom edge of plot
    uint32_t yTop = 40;    // top edge of plot
    uint32_t xStep = 3;    // Distance between points on the X-axis
    uint32_t plotWidth = 300;
    

    // Axis scaling and labels
    uint32_t yMin = 0, yMax = 100, yScale = 1;
    const char *yLabel = "";
    const char *xLabel = "Time (s)";

    switch (plotType)
    {
    case PLOT_LIGHT:
        yMin = 0;
        yMax = 500;
        yScale = 1; // 1 unit per lux
        yLabel = "Lux";
        xStep = 3; // 3 pixels per time unit
        break;
    case PLOT_ACCEL:
        yMin = 0;
        yMax = 100; // not sure what the max accel value is, so using 500 as a placeholder
        yScale = 1; 
        yLabel = "g";
        xStep = 1;
        break;
    case PLOT_RPM:
        yMin = 0;
        yMax = 2500;
        yScale = 1; 
        yLabel = "RPM";
        break;
    case PLOT_POWER:
        yMin = 0;
        yMax = 1000;
        yScale = 1; 
        yLabel = "W";
        break;
    default:
        break;
    }
    uint32_t maxPoints = plotWidth / xStep;

    if (dataSize >= maxPoints)
    {
        // Clear the plotting area
        tRectangle sRect = {xStart, yTop, xStart + plotWidth, yStart};
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);

        // Draw the axes
        GrContextForegroundSet(&sContext, ClrWhite);
        GrLineDraw(&sContext, xStart, yStart, xStart + plotWidth, yStart); // X-axis
        GrLineDraw(&sContext, xStart, yTop, xStart, yStart);               // Y-axis

        // Save the latest value
        uint32_t lastValue = data[dataSize - 1];

        // Reset buffer and index, and add the new value as the first point
        for (uint32_t i = 0; i < maxPoints; i++)
            data[i] = 0;
        data[0] = lastValue;
        switch (plotType)
        {
        case PLOT_LIGHT:
            g_ui32LightDataIndex = 1;
            g_ui32LightDataCount = 1;
            break;
        case PLOT_ACCEL:
            g_ui32AccelDataIndex = 1;
            g_ui32AccelDataCount = 1;
            break;
        default:
            break;
        }
        return;
    }

    // Draw only the new segment
    if (dataSize > 1)
    {
        uint32_t i = dataSize - 1;
        uint32_t x1 = xStart + (i - 1) * xStep;
        uint32_t x2 = xStart + i * xStep;

        // Scale and clamp Y values to plot area
        uint32_t y1 = yStart - ((data[i - 1] - yMin) * (yStart - yTop)) / (yMax - yMin);
        uint32_t y2 = yStart - ((data[i] - yMin) * (yStart - yTop)) / (yMax - yMin);

        if (y1 < yTop)
            y1 = yTop;
        if (y1 > yStart)
            y1 = yStart;
        if (y2 < yTop)
            y2 = yTop;
        if (y2 > yStart)
            y2 = yStart;

        if (filtered) GrContextForegroundSet(&sContext, ClrRed);
        else GrContextForegroundSet(&sContext, ClrBlue);
        GrLineDraw(&sContext, x1, y1, x2, y2);
    }
    // Add labels for the axes
    GrContextForegroundSet(&sContext, ClrWhite);
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDraw(&sContext, xLabel, -1, xStart + 125, yStart + 6, false); // X-axis label
    GrStringDraw(&sContext, yLabel, -1, 20, 33, false);                    // Y-axis label
    // Draw Y axis min/max labels
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    char yMinStr[8], yMaxStr[8];
    usprintf(yMinStr, "%u", yMin);
    usprintf(yMaxStr, "%u", yMax);
    GrContextForegroundSet(&sContext, ClrRed);
    GrStringDraw(&sContext, yMinStr, -1, xStart - 8, yStart - 8, false);
    GrStringDraw(&sContext, yMaxStr, -1, xStart - 8, yTop - 15, false);
    // Draw X axis labels
    GrStringDraw(&sContext, "20", -1, 305, yStart + 6, false);
}
void xTimerHandler(void)
{
    static int count = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    if (count == 30)
    {
        UpdateTime();
        count = 0;
    }
    count++;
    xSemaphoreGiveFromISR(xSemaphoreTimer0, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
