#ifndef BUTTON_TASK_H
#define BUTTON_TASK_H

#include <stdint.h>
#include <stdbool.h>

// Function prototypes
void vCreateButtonTask(void);
void xButtonsHandler(void);
void prvConfigureButton(void);
#endif // BUTTON_TASK_H