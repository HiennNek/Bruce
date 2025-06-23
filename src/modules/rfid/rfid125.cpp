/**
 * @file rfid125.cpp
 * @author Rennan Cockles (https://github.com/rennancockles)
 * @brief Read RFID 125kHz tags
 * @version 0.1
 * @date 2024-08-13
 */

// RFID 125KHz emulation emplemented by @HiennNek

#include "rfid125.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include <globals.h>

RFID125::RFID125() {
    _initial_state = READ_MODE;
    setup();
}

RFID125::RFID125(RFID125_State initial_state) {
    if (initial_state == SAVE_MODE) { initial_state = READ_MODE; }
    _initial_state = initial_state;
    setup();
}

void RFID125::setup() {
    _stream = new HardwareSerial(1);
    _stream->begin(9600, SERIAL_8N1, RFID125_RX_PIN, RFID125_TX_PIN);

    set_state(_initial_state);
    delay(500);
    return loop();
}

void RFID125::loop() {
    while (1) {
        if (check(EscPress)) {
            _stream->end();
            returnToMenu = true;
            break;
        }

        if (check(SelPress)) { select_state(); }

        switch (_current_state) {
            case READ_MODE: read_card(); break;
            case LOAD_MODE: load_file(); break;
            // case CLONE_MODE:
            //     clone_card();
            //     break;
            // case WRITE_MODE:
            //     write_data();
            //     break;
            // case WRITE_NDEF_MODE:
            //     write_ndef_data();
            //     break;
            // case ERASE_MODE:
            //     erase_card();
            //     break;
            case SAVE_MODE: save_file(); break;
        }
    }
}

void RFID125::select_state() {
    options = {};
    if (_tag_read) {
        //     options.push_back({"Clone UID",  [=]() { set_state(CLONE_MODE); }});
        //     options.push_back({"Write data", [=]() { set_state(WRITE_MODE); }});
        options.push_back({"Save file", [=]() { set_state(SAVE_MODE); }});
    }
    options.push_back({"Read tag", [=]() { set_state(READ_MODE); }});
    options.push_back({"Load file", [=]() { set_state(LOAD_MODE); }});
    // options.push_back({"Write NDEF", [=]() { set_state(WRITE_NDEF_MODE); }});
    // options.push_back({"Erase tag",  [=]() { set_state(ERASE_MODE); }});
    loopOptions(options);
}

void RFID125::set_state(RFID125_State state) {
    _current_state = state;
    display_banner();
    switch (state) {
        case READ_MODE: _tag_read = false; break;
        case LOAD_MODE: _tag_read = false; break;
        // case CLONE_MODE:
        //     padprintln("New UID: " + printableUID.uid);
        //     padprintln("SAK: " + printableUID.sak);
        //     padprintln("");
        //     break;
        // case WRITE_MODE:
        //     if (!pageReadSuccess) padprintln("[!] Data blocks are incomplete");
        //     padprintln(String(dataPages) + " pages of data to write");
        //     padprintln("");
        //     break;
        // case WRITE_NDEF_MODE:
        //     _ndef_created = false;
        //     break;
        case SAVE_MODE:
            // case ERASE_MODE:
            break;
    }
    delay(300);
}

void RFID125::cls() {
    drawMainBorder();
    tft.setCursor(10, 28);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
}

void RFID125::display_banner() {
    cls();
    tft.setTextSize(FM);
    padprintln("RFID 125kHz");
    tft.setTextSize(FP);

    switch (_current_state) {
        case READ_MODE:
            padprintln("             READ MODE");
            padprintln("             ---------");
            break;
        case LOAD_MODE:
            padprintln("             LOAD MODE");
            padprintln("             ---------");
            break;
        // case CLONE_MODE:
        //     padprintln("            CLONE MODE");
        //     padprintln("            ----------");
        //     break;
        // case ERASE_MODE:
        //     padprintln("            ERASE MODE");
        //     padprintln("            ----------");
        //     break;
        // case WRITE_MODE:
        //     padprintln("       WRITE DATA MODE");
        //     padprintln("       ---------------");
        //     break;
        // case WRITE_NDEF_MODE:
        //     padprintln("       WRITE NDEF MODE");
        //     padprintln("       ---------------");
        //     break;
        case SAVE_MODE:
            padprintln("             SAVE MODE");
            padprintln("             ---------");
            break;
    }

    tft.setTextSize(FP);
    padprintln("");
    padprintln("Press [OK] to change mode.");
    padprintln("");
    padprintln("");
}

void RFID125::dump_card_details() { padprintln("Tag Data: " + _printable_data); }

void RFID125::read_card() {
    if (!read_card_data()) return;

    display_banner();
    format_data();
    dump_card_details();
    _tag_read = true;
    delay(500);

    clear_stream();
}

bool RFID125::read_card_data() {
    char buff[RFID125_PACKET_SIZE];
    uint8_t checksum;
    uint32_t tag_id;

    if (!_stream) return false;

    if (!_stream->available()) return false;

    /* if a packet doesn't begin with the right byte, remove that byte */
    if (_stream->peek() != RFID125_START_MARK && _stream->read()) return false;

    /* if read a packet with the wrong size, drop it */
    if (RFID125_PACKET_SIZE != _stream->readBytes(buff, RFID125_PACKET_SIZE)) return false;

    /* if a packet doesn't end with the right byte, drop it */
    if (buff[13] != RFID125_END_MARK) return false;

    for (int i = 0; i < RFID125_PACKET_SIZE; i++) _tag_data[i] = buff[i];

    /* add null and parse checksum */
    buff[13] = 0;
    checksum = strtol(buff + 11, NULL, 16);
    /* add null and parse tag_id */
    buff[11] = 0;
    tag_id = strtol(buff + 3, NULL, 16);
    /* add null and parse version (needs to be xored with checksum) */
    buff[3] = 0;
    checksum ^= strtol(buff + 1, NULL, 16);

    /* xore the tag_id and validate checksum */
    for (uint8_t i = 0; i < 32; i += 8) checksum ^= ((tag_id >> i) & 0xFF);
    if (checksum) return false;

    return true;
}

void RFID125::clear_stream() {
    while (_stream->available()) _stream->read();
}

void RFID125::save_file() {
    String data = _printable_data;
    data.replace(" ", "");
    String filename = keyboard(data, 30, "File name:");

    display_banner();

    if (write_file(filename)) {
        displaySuccess("File saved.");
    } else {
        displayError("Error writing file.");
    }
    delay(1000);
    set_state(READ_MODE);
}

bool RFID125::write_file(String filename) {
    FS *fs;
    if (!getFsStorage(fs)) return false;

    if (!(*fs).exists("/BruceRFID")) (*fs).mkdir("/BruceRFID");
    if ((*fs).exists("/BruceRFID/" + filename + ".rfidlf")) {
        int i = 1;
        filename += "_";
        while ((*fs).exists("/BruceRFID/" + filename + String(i) + ".rfidlf")) i++;
        filename += String(i);
    }
    File file = (*fs).open("/BruceRFID/" + filename + ".rfidlf", FILE_WRITE);

    if (!file) { return false; }

    String file_data = "";
    for (byte i = 0; i < RFID125_PACKET_SIZE; i++) {
        file_data += _tag_data[i] < 0x10 ? " 0" : " ";
        file_data += String(_tag_data[i], HEX);
    }
    file_data.trim();
    file_data.toUpperCase();

    file.println("Filetype: Bruce RFID 125kHz File");
    file.println("Version 1");
    file.println("DATA: " + file_data);
    file.println("ASCII: " + _printable_data);
    file.println("CHECKSUM: " + _printable_checksum);

    file.close();
    delay(100);
    return true;
}

void RFID125::format_data() {
    _printable_data = "";
    for (byte i = 1; i < RFID125_PACKET_SIZE - 3; i += 2) {
        _printable_data += String(_tag_data[i]);
        _printable_data += String(_tag_data[i + 1]);
        _printable_data += " ";
    }
    _printable_data.trim();
    _printable_data.toUpperCase();

    _printable_checksum = String(_tag_data[RFID125_PACKET_SIZE - 3]);
    _printable_checksum += String(_tag_data[RFID125_PACKET_SIZE - 2]);
    _printable_checksum.trim();
    _printable_checksum.toUpperCase();
}

void RFID125::load_file() {
    display_banner();

    String filepath;
    File file;
    FS *fs;

    if (!getFsStorage(fs)) {
        displayError("Error accessing storage");
        delay(1000);
        set_state(READ_MODE);
        return;
    }

    if (!(*fs).exists("/BruceRFID")) (*fs).mkdir("/BruceRFID");
    filepath = loopSD(*fs, true, "RFIDLF", "/BruceRFID");
    file = fs->open(filepath, FILE_READ);

    if (!file) {
        displayError("Error opening file");
        delay(1000);
        set_state(READ_MODE);
        return;
    }

    String line;
    String strData;
    _printable_data = "";

    while (file.available()) {
        line = file.readStringUntil('\n');
        strData = line.substring(line.indexOf(":") + 1);
        strData.trim();
        if (line.startsWith("ASCII:")) _printable_data = strData;
    }

    file.close();
    delay(100);

    if (_printable_data == "") {
        displayError("Invalid file format");
        delay(1000);
        set_state(READ_MODE);
        return;
    }

    displaySuccess("File loaded");
    delay(1000);
    _tag_read = true;

    options = {
        {"Emulate tag", [=]() { emulate_tag(); }},
    };

    loopOptions(options);
    set_state(READ_MODE);
}

void RFID125::emulate_tag() { // @HiennNek
    display_banner();
    padprintln("Emulating tag: " + _printable_data);
    padprintln("");
    padprintln("Press [ESC] to stop");

    // Convert card ID and vendor to bytes
    String hex_str = _printable_data;
    hex_str.replace(" ", "");
    uint8_t tag_bytes[5];
    for (int i = 0; i < 5; i++) {
        tag_bytes[i] = strtol(hex_str.substring(i * 2, i * 2 + 2).c_str(), NULL, 16);
    }

    // Since the RDM6300 doesnt use RX pin, we can use it for emulation
    pinMode(RFID125_RX_PIN, OUTPUT);
    digitalWrite(RFID125_RX_PIN, LOW);

    // Code borrowed from Crypter/ESP-RFID
    while (!check(EscPress)) {
        // Some math stuff -_-

        uint8_t data[64];
        uint8_t value[10];

        // Convert vendor and ID to nibbles
        value[0] = (tag_bytes[0] >> 4) & 0xF;
        value[1] = tag_bytes[0] & 0xF;

        uint32_t ID = (tag_bytes[1] << 24) | (tag_bytes[2] << 16) | (tag_bytes[3] << 8) | tag_bytes[4];
        for (int i = 1; i < 8; i++) { value[i + 2] = (ID >> (28 - i * 4)) & 0xF; }

        // Data frame
        for (int i = 0; i < 9; i++) data[i] = 1; // Header

        for (int i = 0; i < 10; i++) { // Data + parity
            for (int j = 0; j < 4; j++) { data[9 + i * 5 + j] = value[i] >> (3 - j) & 1; }
            // Calculate parity bit
            data[9 + i * 5 + 4] =
                (data[9 + i * 5 + 0] + data[9 + i * 5 + 1] + data[9 + i * 5 + 2] + data[9 + i * 5 + 3]) % 2;
        }

        // Checksum
        for (int i = 0; i < 4; i++) {
            int checksum = 0;
            for (int j = 0; j < 10; j++) { checksum += data[9 + i + j * 5]; }
            data[i + 59] = checksum % 2;
        }
        data[63] = 0; // Footer

        // This is called manchester encoding :D
        for (int i = 0; i < 15; i++) { // Repeat signal
            for (int j = 0; j < 64; j++) {
                digitalWrite(RFID125_RX_PIN, data[j] ? HIGH : LOW);
                delayMicroseconds(255);
                digitalWrite(RFID125_RX_PIN, data[j] ? LOW : HIGH);
                delayMicroseconds(255);
            }
        }
        delay(10);
    }

    digitalWrite(RFID125_RX_PIN, LOW);
    displayInfo("Emulation stopped");
    delay(1000);
}
