#include "FileMenu.h"
#include "core/display.h"
#include "core/sd_functions.h"
#include "modules/others/webInterface.h"
#include "core/utils.h"

void FileMenu::optionsMenu() {
    options = {
        {"SD Card",      [=]() { loopSD(SD); }},
        {"LittleFS",     [=]() { loopSD(LittleFS); }},
        {"WebUI",        [=]() { loopOptionsWebUi(); }},
        {"Main Menu",    [=]() { backToMenu(); }},
    };

    loopOptions(options,false,true,"Files");
}

void FileMenu::drawIcon(float scale) {
    clearIconArea();

    int iconW = scale * 32;
    int iconH = scale * 48;

    if (iconW % 2 != 0) iconW++;
    if (iconH % 2 != 0) iconH++;

    int foldSize = iconH/4;
    int iconX = iconCenterX - iconW/2;
    int iconY = iconCenterY - iconH/2;
    int iconDelta = 10;

    // Files
    tft.drawRect(iconX+iconDelta, iconY-iconDelta, iconW, iconH, fzerofirmwareConfig.priColor);

    tft.fillRect(iconX, iconY, iconW, iconH, fzerofirmwareConfig.bgColor);
    tft.drawRect(iconX, iconY, iconW, iconH, fzerofirmwareConfig.priColor);

    tft.fillRect(iconX-iconDelta, iconY+iconDelta, iconW, iconH, fzerofirmwareConfig.bgColor);
    tft.drawRect(iconX-iconDelta, iconY+iconDelta, iconW, iconH, fzerofirmwareConfig.priColor);

    // Erase corners
    tft.fillRect(iconX+iconDelta+iconW-foldSize, iconY-iconDelta-1, foldSize, 2, fzerofirmwareConfig.bgColor);
    tft.fillRect(iconX+iconDelta+iconW-1, iconY-iconDelta, 2, foldSize, fzerofirmwareConfig.bgColor);

    tft.fillRect(iconX+iconW-foldSize, iconY-1, foldSize, 2, fzerofirmwareConfig.bgColor);
    tft.fillRect(iconX+iconW-1, iconY, 2, foldSize, fzerofirmwareConfig.bgColor);

    tft.fillRect(iconX-iconDelta+iconW-foldSize, iconY+iconDelta-1, foldSize, 2, fzerofirmwareConfig.bgColor);
    tft.fillRect(iconX-iconDelta+iconW-1, iconY+iconDelta, 2, foldSize, fzerofirmwareConfig.bgColor);

    // Folds
    tft.drawTriangle(
        iconX+iconDelta+iconW-foldSize, iconY-iconDelta,
        iconX+iconDelta+iconW-foldSize, iconY-iconDelta+foldSize-1,
        iconX+iconDelta+iconW-1, iconY-iconDelta+foldSize-1,
        fzerofirmwareConfig.priColor
    );
    tft.drawTriangle(
        iconX+iconW-foldSize, iconY,
        iconX+iconW-foldSize, iconY+foldSize-1,
        iconX+iconW-1, iconY+foldSize-1,
        fzerofirmwareConfig.priColor
    );
    tft.drawTriangle(
        iconX-iconDelta+iconW-foldSize, iconY+iconDelta,
        iconX-iconDelta+iconW-foldSize, iconY+iconDelta+foldSize-1,
        iconX-iconDelta+iconW-1, iconY+iconDelta+foldSize-1,
        fzerofirmwareConfig.priColor
    );
}
