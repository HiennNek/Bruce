#include "RFIDMenu.h"
#include "core/display.h"
#include "core/settings.h"
#include "modules/rfid/tag_o_matic.h"
#include "modules/rfid/rfid125.h"
#include "modules/rfid/chameleon.h"
#include "modules/rfid/amiibo.h"
#include "modules/rfid/pn532ble.h"
#include "core/utils.h"

void RFIDMenu::optionsMenu() {
    options = {
        {"Read tag",    [=]()  { TagOMatic(); }},
        {"Read 125kHz", [=]()  { RFID125(); }},
        {"Scan tags",   [=]()  { TagOMatic(TagOMatic::SCAN_MODE); }},
        {"Load file",   [=]()  { TagOMatic(TagOMatic::LOAD_MODE); }},
        {"Erase data",  [=]()  { TagOMatic(TagOMatic::ERASE_MODE); }},
        {"Write NDEF",  [=]()  { TagOMatic(TagOMatic::WRITE_NDEF_MODE); }},
        {"Amiibolink",  [=]()  { Amiibo(); }},
        {"Chameleon",   [=]()  { Chameleon(); }},
        {"PN532 BLE",   [=]()  { Pn532ble(); }},
        {"Config",      [=]()  { configMenu(); }},
        {"Main Menu",   [=]()  { backToMenu(); }},
    };

    delay(200);

    String txt = "RFID";
    if(fzerofirmwareConfig.rfidModule==M5_RFID2_MODULE)        txt+=" (RFID2)";
    else if(fzerofirmwareConfig.rfidModule==PN532_I2C_MODULE)  txt+=" (PN532-I2C)";
    else if(fzerofirmwareConfig.rfidModule==PN532_SPI_MODULE)  txt+=" (PN532-SPI)";
    loopOptions(options,false,true,txt);
}

void RFIDMenu::configMenu() {
    options = {
        {"RFID Module",   [=]() { setRFIDModuleMenu(); }},
        {"Add MIF Key",   [=]() { addMifareKeyMenu(); }},
        {"Back",          [=]() { optionsMenu(); }},
    };

    loopOptions(options,false,true,"RFID Config");
}

void RFIDMenu::drawIcon(float scale) {
    clearIconArea();

    int iconSize = scale * 70;
    int iconRadius = scale * 7;
    int deltaRadius = scale * 10;

    if (iconSize % 2 != 0) iconSize++;

    tft.drawRoundRect(
        iconCenterX - iconSize/2,
        iconCenterY - iconSize/2,
        iconSize,
        iconSize,
        iconRadius,
        fzerofirmwareConfig.priColor
    );
    tft.fillRect(
        iconCenterX - iconSize/2,
        iconCenterY,
        iconSize/2,
        iconSize/2,
        fzerofirmwareConfig.bgColor
    );

    tft.drawCircle(
        iconCenterX - iconSize/2 + deltaRadius,
        iconCenterY + iconSize/2 - deltaRadius,
        iconRadius,
        fzerofirmwareConfig.priColor
    );

    tft.drawArc(
        iconCenterX - iconSize/2 + deltaRadius,
        iconCenterY + iconSize/2 - deltaRadius,
        2.5*iconRadius, 2*iconRadius,
        180,270,
        fzerofirmwareConfig.priColor,
        fzerofirmwareConfig.bgColor
    );
    tft.drawArc(
        iconCenterX - iconSize/2 + deltaRadius,
        iconCenterY + iconSize/2 - deltaRadius,
        2.5*iconRadius + deltaRadius, 2*iconRadius + deltaRadius,
        180,270,
        fzerofirmwareConfig.priColor,
        fzerofirmwareConfig.bgColor
    );
    tft.drawArc(
        iconCenterX - iconSize/2 + deltaRadius,
        iconCenterY + iconSize/2 - deltaRadius,
        2.5*iconRadius + 2*deltaRadius, 2*iconRadius + 2*deltaRadius,
        180,270,
        fzerofirmwareConfig.priColor,
        fzerofirmwareConfig.bgColor
    );
}