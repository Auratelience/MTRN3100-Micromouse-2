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
#include "oledDisplay.h"

struct OLEDValue {
    const char* label = nullptr;
    etl::delegate<float()> value;
};

// A page of labelled scalars, one or two columns deep.
//
// Was OLED, and owned the panel. It now borrows an OLEDDisplay, because
// OLEDMap and OLEDPath draw to the same panel and only one thing can own it.
class OLEDValues {
    public:

    template <size_t N>
    OLEDValues(OLEDDisplay& display, const std::array<OLEDValue, N>& values) :
        display(display),
        values(clampSpan(etl::span<const OLEDValue>(values.data(), values.size()))) {}

    void update() {
        if (!display.ready()) return;
        if (!display.due()) return;

        Adafruit_SSD1306& g = display.gfx();
        g.clearDisplay();
        g.setTextSize(OLED_TEXT_SIZE);
        g.setTextColor(SSD1306_WHITE);

        const uint8_t slots = maxVisibleValues();
        for (uint8_t i = 0; i < slots; ++i) {
            drawValue(i, values[i]);
        }

        g.display();
    }

    template <size_t N>
    void setValues(const std::array<OLEDValue, N>& newValues) {
        values = clampSpan(etl::span<const OLEDValue>(newValues.data(), newValues.size()));
    }

    private:

    OLEDDisplay& display;
    etl::span<const OLEDValue> values;

    static etl::span<const OLEDValue> clampSpan(etl::span<const OLEDValue> s) {
        return s.first(etl::min<size_t>(s.size(), static_cast<size_t>(OLED_MAX_VALUES)));
    }

    uint8_t rowCount() const {
        return display.height() / OLED_TEXT_HEIGHT;
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
        Adafruit_SSD1306& g       = display.gfx();
        const uint8_t rows        = rowCount();
        const uint8_t column      = index / rows;
        const uint8_t row         = index % rows;
        const uint8_t columnWidth = display.width() / columnCount();
        const uint8_t labelChars =
            (columnCount() == 1) ? OLED_ONE_COLUMN_LABEL_CHARS : OLED_TWO_COLUMN_LABEL_CHARS;
        const uint8_t x = column * columnWidth;
        const uint8_t y = row * OLED_TEXT_HEIGHT;

        g.setCursor(x, y);
        printLabel(item.label, labelChars);

        g.setCursor(x + ((labelChars + 1) * OLED_CHAR_WIDTH), y);
        if (!item.value.is_valid()) {
            g.print(F("--"));
            return;
        }

        g.print(
            item.value(), columnCount() == 1 ? OLED_ONE_COLUMN_DECIMALS : OLED_TWO_COLUMN_DECIMALS
        );
    }

    void printLabel(const char* label, uint8_t maxChars) {
        Adafruit_SSD1306& g = display.gfx();
        if (label == nullptr) {
            g.print(F("?"));
            return;
        }

        for (uint8_t i = 0; label[i] != '\0' && i < maxChars; ++i) {
            g.print(label[i]);
        }
        g.print(F(":"));
    }
};
