#include "src/zController.h"
#include "include/definitions.h"
#include "include/ErrorDef.h"
#include "src/zController.h"

ZController* controller;

void setup() {
    LOG_LOG("Instrument Awareness Engine v0.1");

    controller = ZController::create();
    Error_t e = controller->init();

    if (e != kNoError) {
        LOG_ERROR("Error initializing controller. code: %i", (int) e);
        while (1) {}
    }

}

void loop() {
    controller->poll();
    delay(1);
}
