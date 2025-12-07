/**
 * @file colors.hpp
 *
 * @brief CAuDri - Color Definitions and Conversions
 *
 * This file defines a Color struct for representing colors in RGB format, along with functions for converting between RGB and HSV color spaces.
 */
#pragma once

#include <algorithm>
#include <cstdint>

// Predefined Colors
#define COLOR_OFF Color(0, 0, 0)

#define COLOR_RED Color(255, 0, 0)
#define COLOR_GREEN Color(0, 255, 0)
#define COLOR_BLUE Color(0, 0, 255)
#define COLOR_YELLOW Color(255, 255, 0)
#define COLOR_CYAN Color(0, 255, 255)
#define COLOR_MAGENTA Color(255, 0, 255)
#define COLOR_WHITE Color(255, 255, 255)
#define COLOR_BLACK Color(0, 0, 0)
#define COLOR_ORANGE Color(255, 165, 0)
#define COLOR_PURPLE Color(128, 0, 128)
#define COLOR_PINK Color(200, 0, 150)
#define COLOR_BROWN Color(165, 42, 42)

/**
 * @brief Color representation for RGB and HSV colors
 *
 * This struct represents a color in RGB format with red, green, and blue components.
 * It also provides a static method to convert HSV values to RGB.
 *
 * Can be created using:
 * - Color(r, g, b) for RGB colors
 * - Color::fromRGB(r, g, b) for RGB colors
 * - Color::fromHSV(h, s, v) for HSV colors
 */
struct Color {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;

    Color() = default;
    Color(uint8_t r, uint8_t g, uint8_t b) : red(r), green(g), blue(b) {}

    bool operator==(const Color& other) const { return red == other.red && green == other.green && blue == other.blue; }
    bool operator!=(const Color& other) const { return !(*this == other); }

    /**
     * @brief Create a Color from RGB values
     *
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     *
     * @return Color The created Color object
     */
    static Color fromRGB(uint8_t r, uint8_t g, uint8_t b) { return Color(r, g, b); }

    /**
     * @brief Create a Color from HSV values
     *
     * @param h Hue component (0-359)
     * @param s Saturation component (0-255)
     * @param v Value component (0-255)
     *
     * @return Color The created Color object
     */
    static Color fromHSV(uint16_t h, uint8_t s, uint8_t v) {
        Color color;

        if (s == 0) {
            color.red = color.green = color.blue = v;
            return color;
        }

        uint16_t region = h / 60;
        uint16_t remainder = (h - (region * 60)) * 255 / 60;

        uint8_t p = (v * (255 - s)) / 255;
        uint8_t q = (v * (255 - ((s * remainder) / 255))) / 255;
        uint8_t t = (v * (255 - ((s * (255 - remainder)) / 255))) / 255;

        switch (region) {
            case 0:
                return Color(v, t, p);
            case 1:
                return Color(q, v, p);
            case 2:
                return Color(p, v, t);
            case 3:
                return Color(p, q, v);
            case 4:
                return Color(t, p, v);
            case 5:
            default:
                return Color(v, p, q);
        }
    }

    /**
     * @brief Convert the Color from RGB to HSV
     *
     * @param h Pointer to store the Hue component (0-359)
     * @param s Pointer to store the Saturation component (0-255)
     * @param v Pointer to store the Value component (0-255)
     */
    void toHSV(uint16_t* h, uint8_t* s, uint8_t* v) {
        // Convert RGB to HSV
        uint8_t max = std::max({red, green, blue});
        uint8_t min = std::min({red, green, blue});
        *v = max;

        if (max == 0) {
            *s = 0;
            *h = 0;
            return;
        }

        *s = ((max - min) * 255) / max;

        if (max == red) {
            *h = ((green - blue) * 60) / (max - min);
        } else if (max == green) {
            *h = 120 + ((blue - red) * 60) / (max - min);
        } else {
            *h = 240 + ((red - green) * 60) / (max - min);
        }

        if (*h < 0) {
            *h += 360;
        }
    }
};