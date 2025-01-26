#include "RFMenu.h"
#include "core/display.h"
#include "core/settings.h"
#include "modules/rf/rf.h"
#include "core/utils.h"

void RFMenu::optionsMenu() {
    options = {
        {"Scan/copy",       [=]() { rf_scan_copy(); }},
        {"Custom SubGhz",   [=]() { otherRFcodes(); }},
        {"Spectrum",        [=]() { rf_spectrum(); }}, //@IncursioHack
        {"SquareWave Spec", [=]() { rf_SquareWave(); }}, //@Pirata
        {"Jammer Itmt",     [=]() { rf_jammerIntermittent(); }}, //@IncursioHack
        {"Jammer Full",     [=]() { rf_jammerFull(); }}, //@IncursioHack
        {"Jammer All Freq", [=]() { rf_jammerAllFreq(); }}, //@HiennNek
        {"Config",          [=]() { configMenu(); }},
        {"Main Menu",       [=]() { backToMenu(); }},
    };

    delay(200);
    String txt = "Radio Frequency";
    if(fzerofirmwareConfig.rfModule==CC1101_SPI_MODULE) txt+=" (CC1101)"; // Indicates if CC1101 is connected
    else txt+=" Tx: " + String(fzerofirmwareConfig.rfTx) + " Rx: " + String(fzerofirmwareConfig.rfRx);

    loopOptions(options,false,true,txt);
}

void RFMenu::configMenu() {
    options = {
        {"RF TX Pin",     [=]() { gsetRfTxPin(true); }},
        {"RF RX Pin",     [=]() { gsetRfRxPin(true); }},
        {"RF Module",     [=]() { setRFModuleMenu(); }},
        {"RF Frequency",  [=]() { setRFFreqMenu();   }},
        {"Back",          [=]() { optionsMenu(); }},
    };

    loopOptions(options,false,true,"RF Config");
}

void RFMenu::drawIcon(float scale) {
    clearIconArea();

    int radius = scale * 7;
    int deltaRadius = scale * 10;
    int triangleSize = scale * 30;

    if (triangleSize % 2 != 0) triangleSize++;

    // Body
    tft.fillCircle(iconCenterX, iconCenterY - radius, radius, fzerofirmwareConfig.priColor);
    tft.fillTriangle(
        iconCenterX, iconCenterY,
        iconCenterX - triangleSize/2, iconCenterY + triangleSize,
        iconCenterX + triangleSize/2, iconCenterY + triangleSize,
        fzerofirmwareConfig.priColor
    );

    // Left Arcs
    tft.drawArc(
        iconCenterX, iconCenterY - radius,
        2.5*radius, 2*radius,
        40, 140,
        fzerofirmwareConfig.priColor, fzerofirmwareConfig.bgColor
    );
    tft.drawArc(
        iconCenterX, iconCenterY - radius,
        2.5*radius + deltaRadius, 2*radius + deltaRadius,
        40, 140,
        fzerofirmwareConfig.priColor, fzerofirmwareConfig.bgColor
    );
    tft.drawArc(
        iconCenterX, iconCenterY - radius,
        2.5*radius + 2*deltaRadius, 2*radius + 2*deltaRadius,
        40, 140,
        fzerofirmwareConfig.priColor, fzerofirmwareConfig.bgColor
    );

    // Right Arcs
    tft.drawArc(
        iconCenterX, iconCenterY - radius,
        2.5*radius, 2*radius,
        220, 320,
        fzerofirmwareConfig.priColor, fzerofirmwareConfig.bgColor
    );
    tft.drawArc(
        iconCenterX, iconCenterY - radius,
        2.5*radius + deltaRadius, 2*radius + deltaRadius,
        220, 320,
        fzerofirmwareConfig.priColor, fzerofirmwareConfig.bgColor
    );
    tft.drawArc(
        iconCenterX, iconCenterY - radius,
        2.5*radius + 2*deltaRadius, 2*radius + 2*deltaRadius,
        220, 320,
        fzerofirmwareConfig.priColor, fzerofirmwareConfig.bgColor
    );
}
