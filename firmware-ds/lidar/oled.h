// SSD1306
//
// Zimmy Levi z5587840

#pragma once

#include <array>

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include <etl/span.h>
#include <etl/delegate.h>
#include <etl/algorithm.h>

#include "constants.h"

struct OLEDValue {
    const char* label = nullptr;
    etl::delegate<float()> value;
};

class OLED {
    public:

    template <size_t N>
    explicit OLED(
        const std::array<OLEDValue, N>& values,
        uint8_t width   = OLED_WIDTH,
        uint8_t height  = OLED_HEIGHT,
        uint8_t address = OLED_ADDRESS,
        int8_t resetPin = OLED_NO_RESET_PIN
    ) :
        display(width, height, &Wire, resetPin),
        values(clampSpan(etl::span<const OLEDValue>(values.data(), values.size()))),
        width(width),
        height(height),
        address(address) {}

    bool init() {
        if (!display.begin(SSD1306_SWITCHCAPVCC, address)) {
            initialized = false;
            return false;
        }

        display.clearDisplay();
        display.setTextSize(OLED_TEXT_SIZE);
        display.setTextColor(SSD1306_WHITE);
        display.cp437(true);
        display.display();
        initialized = true;
        return true;
    }

    void update() {
        if (!initialized) return;
        const unsigned long now = millis();
        if (now - lastRefreshMs < OLED_REFRESH_MS) return;
        lastRefreshMs = now;

        display.clearDisplay();
        display.setTextSize(OLED_TEXT_SIZE);
        display.setTextColor(SSD1306_WHITE);

        const uint8_t slots = maxVisibleValues();
        for (uint8_t i = 0; i < slots; ++i) {
            drawValue(i, values[i]);
        }

        display.display();
    }

    void clear() {
        if (!initialized) return;
        display.clearDisplay();
        display.display();
    }

    template <size_t N>
    void setValues(const std::array<OLEDValue, N>& newValues) {
        values = clampSpan(etl::span<const OLEDValue>(newValues.data(), newValues.size()));
    }

    private:

    Adafruit_SSD1306 display;
    etl::span<const OLEDValue> values;
    uint8_t width;
    uint8_t height;
    uint8_t address;
    bool initialized            = false;
    unsigned long lastRefreshMs = 0;

    static etl::span<const OLEDValue> clampSpan(etl::span<const OLEDValue> s) {
        return s.first(etl::min<size_t>(s.size(), static_cast<size_t>(OLED_MAX_VALUES)));
    }

    uint8_t rowCount() const {
        return height / OLED_TEXT_HEIGHT;
    }

    uint8_t columnCount() const {
        return (values.size() > rowCount()) ? 2 : 1;
    }

    uint8_t maxVisibleValues() const {
        uint8_t visible = rowCount() * columnCount();
        visible         = etl::min<uint8_t>(visible, OLED_MAX_VALUES);
        return etl::min<uint8_t>(static_cast<uint8_t>(values.size()), visible);
    }

    void drawValue(uint8_t index, const OLEDValue& item) {
        const uint8_t rows        = rowCount();
        const uint8_t column      = index / rows;
        const uint8_t row         = index % rows;
        const uint8_t columnWidth = width / columnCount();
        const uint8_t labelChars =
            (columnCount() == 1) ? OLED_ONE_COLUMN_LABEL_CHARS : OLED_TWO_COLUMN_LABEL_CHARS;
        const uint8_t x = column * columnWidth;
        const uint8_t y = row * OLED_TEXT_HEIGHT;

        display.setCursor(x, y);
        printLabel(item.label, labelChars);

        display.setCursor(x + ((labelChars + 1) * OLED_CHAR_WIDTH), y);
        if (!item.value.is_valid()) {
            display.print(F("--"));
            return;
        }

        display.print(
            item.value(), columnCount() == 1 ? OLED_ONE_COLUMN_DECIMALS : OLED_TWO_COLUMN_DECIMALS
        );
    }

    void printLabel(const char* label, uint8_t maxChars) {
        if (label == nullptr) {
            display.print(F("?"));
            return;
        }

        for (uint8_t i = 0; label[i] != '\0' && i < maxChars; ++i) {
            display.print(label[i]);
        }
        display.print(F(":"));
    }
};
