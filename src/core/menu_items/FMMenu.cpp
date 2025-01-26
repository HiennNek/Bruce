#include "FMMenu.h"
#include "core/display.h"
#include "modules/fm/fm.h"
#include "core/utils.h"

void FMMenu::optionsMenu() {
    options = {
//    #if !defined(LITE_VERSION) and defined(FM_SI4713)
      #if defined(FM_SI4713)
        {"Brdcast std",   [=]() { fm_live_run(false); }},
        {"Brdcast rsvd",  [=]() { fm_live_run(true); }},
        {"Brdcast stop",  [=]() { fm_stop(); }},
        {"FM Spectrum",   [=]() { fm_spectrum(); }},
        {"Hijack TA",     [=]() { fm_ta_run(); }},
    #endif
        {"Main Menu",     [=]() { backToMenu(); }}
    };

    loopOptions(options,false,true,"FM");
}

void FMMenu::drawIcon(float scale) {
    clearIconArea();

    int iconW = scale * 80;
    int iconH = scale * 60;

    if (iconW % 2 != 0) iconW++;
    if (iconH % 2 != 0) iconH++;

    int caseH = 5*iconH/6;
    int caseX = iconCenterX - iconW/2;
    int caseY = iconCenterY - iconH/3;

    int btnY = (2*caseY + caseH + iconH/10 + iconH/3) / 2;
    int potX = (2*caseX + iconW + iconW/10 + iconW/2) / 2;
    int potY = caseY + caseH/3 + iconH/10;

    // Case
    tft.drawRoundRect(
        caseX, caseY,
        iconW, caseH,
        iconW/10,
        fzerofirmwareConfig.priColor
    );

    // Potentiometer
    tft.fillCircle(
        potX, potY,
        iconH/7,
        fzerofirmwareConfig.priColor
    );

    // Screen
    tft.drawRect(
        caseX + iconW/10,
        caseY + iconH/10,
        iconW/2, iconH/3,
        fzerofirmwareConfig.priColor
    );

    // Antenna
    tft.drawLine(
        caseX + iconW/10, caseY,
        caseX + iconW/10 + iconH/3, caseY - iconH/6,
        fzerofirmwareConfig.priColor
    );
    tft.fillCircle(
        caseX + iconW/10 + iconH/3,
        caseY - iconH/6,
        iconH/30,
        fzerofirmwareConfig.priColor
    );

    // Buttons
    tft.fillCircle(
        caseX + iconW/10 + iconH/8,
        btnY,
        iconH/12,
        fzerofirmwareConfig.priColor
    );
    tft.fillCircle(
        caseX + iconW/10 + iconW/2 - iconH/8,
        btnY,
        iconH/12,
        fzerofirmwareConfig.priColor
    );
}