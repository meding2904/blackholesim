#include "ui.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>

namespace bh {
namespace {

constexpr float kPanelX = 26.0f;
constexpr float kPanelY = 26.0f;
constexpr float kPanelWidth = 338.0f;
constexpr float kPad = 20.0f;
constexpr float kRowHeight = 34.0f;
constexpr float kSliderTrackOffset = 17.0f;

// Section heights are shared by the layout and the drawing code so the panel
// frame always encloses its content.
constexpr float kPadTop = 18.0f;
constexpr float kPadBottom = 18.0f;
constexpr float kHeaderHeight = 76.0f;
constexpr float kStatusHeight = 141.0f;
constexpr float kDiskHeight = 172.0f;
constexpr float kRenderHeight = 142.0f;
constexpr float kScaleHeight = 112.0f;
constexpr float kLegendHeight = 75.0f;

const Color kPanelFill = {9, 12, 18, 216};
const Color kPanelBorder = {255, 255, 255, 26};
const Color kText = {236, 240, 246, 255};
const Color kTextDim = {138, 150, 166, 255};
const Color kTextFaint = {96, 107, 122, 255};
const Color kAccent = {255, 176, 102, 255};
const Color kAccentSoft = {255, 176, 102, 38};
const Color kTrack = {255, 255, 255, 34};
const Color kHover = {255, 255, 255, 9};

struct Layout {
    Rectangle panel;
    bool showScale = false;
    bool showLegend = false;
};

Layout computeLayout(int screenHeight) {
    Layout layout;
    if (screenHeight >= 790) {
        layout.showScale = true;
        layout.showLegend = true;
    } else if (screenHeight >= 700) {
        layout.showLegend = true;
    }

    float height = kPadTop + kHeaderHeight + kStatusHeight + kDiskHeight + kRenderHeight +
                   kPadBottom;
    if (layout.showScale) height += kScaleHeight;
    if (layout.showLegend) height += kLegendHeight;

    const float available = static_cast<float>(screenHeight) - 2.0f * kPanelY;
    layout.panel = {kPanelX, kPanelY, kPanelWidth, std::min(height, std::max(240.0f, available))};
    return layout;
}

unsigned int idOf(const char* label) {
    uint32_t hash = 2166136261u;
    for (const char* c = label; *c != '\0'; ++c) {
        hash ^= static_cast<uint32_t>(*c);
        hash *= 16777619u;
    }
    return hash;
}

void text(const char* value, float x, float y, float size, Color color, float spacing = 0.4f) {
    DrawTextEx(GetFontDefault(), value, {x, y}, size, spacing, color);
}

float textWidth(const char* value, float size, float spacing = 0.4f) {
    return MeasureTextEx(GetFontDefault(), value, size, spacing).x;
}

void textRight(const char* value, float rightX, float y, float size, Color color,
               float spacing = 0.4f) {
    text(value, rightX - textWidth(value, size, spacing), y, size, color, spacing);
}

void divider(float x, float width, float y) {
    DrawRectangle(static_cast<int>(x), static_cast<int>(y), static_cast<int>(width), 1,
                  {255, 255, 255, 18});
}

void sectionTitle(const char* label, float x, float y) {
    text(label, x, y, 10.0f, kTextFaint, 1.4f);
    DrawRectangle(static_cast<int>(x + textWidth(label, 10.0f, 1.4f) + 10.0f),
                  static_cast<int>(y + 4), 28, 1, {255, 255, 255, 16});
}

}  // namespace

bool Ui::containsMouse() const {
    const Vector2 mouse = GetMousePosition();
    const Layout layout = computeLayout(GetScreenHeight());
    return CheckCollisionPointRec(mouse, layout.panel);
}

void Ui::draw(SimulationParameters& params, const UiReadout& readout) {
    if (!params.showUi) return;

    const Layout layout = computeLayout(GetScreenHeight());
    const float x = layout.panel.x + kPad;
    const float width = layout.panel.width - 2.0f * kPad;
    const float right = x + width;
    const Vector2 mouse = GetMousePosition();

    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) m_activeSlider = 0;

    DrawRectangleRounded(layout.panel, 0.028f, 6, kPanelFill);
    DrawRectangleRoundedLinesEx(layout.panel, 0.028f, 6, 1.0f, kPanelBorder);
    DrawRectangle(static_cast<int>(layout.panel.x), static_cast<int>(layout.panel.y) + 18, 2, 38,
                  kAccent);

    float y = layout.panel.y + kPadTop;

    // ---- header ------------------------------------------------------------
    text("SCHWARZSCHILD", x, y, 20.0f, kText, 4.2f);
    y += 27.0f;
    text("GRAVITATIONAL LENSING", x, y, 10.0f, kAccent, 1.8f);
    y += 14.0f;
    text("REAL-TIME NULL-GEODESIC TRACER", x, y, 10.0f, kTextFaint, 1.2f);
    y += 25.0f;  // 66 total from the panel top padding
    divider(x, width, y);
    y += 10.0f;

    // ---- live readouts -----------------------------------------------------
    char buffer[96];
    const auto readoutRow = [&](const char* label, const char* value, Color valueColor) {
        text(label, x, y, 10.0f, kTextDim, 1.1f);
        textRight(value, right, y, 10.0f, valueColor);
        y += 17.0f;
    };

    readoutRow("MODEL", "SCHWARZSCHILD", kText);
    std::snprintf(buffer, sizeof(buffer), "%.1f r_g", readout.cameraDistance);
    readoutRow("DISTANCE", buffer, kText);
    std::snprintf(buffer, sizeof(buffer), "%.1f deg", readout.inclinationDegrees);
    readoutRow("INCLINATION", buffer, kText);
    readoutRow("DISK", params.paused ? "PAUSED" : "ANIMATING",
               params.paused ? kTextDim : kAccent);
    readoutRow("QUALITY", qualitySettings(params.quality).label, kText);
    std::snprintf(buffer, sizeof(buffer), "%d FPS", readout.fps);
    readoutRow("FRAME", buffer, kText);
    // Distances are in r_g, so only the physical scale depends on the mass.
    char scaleLabel[48];
    std::snprintf(scaleLabel, sizeof(scaleLabel), "1 r_g AT %.0f M_SUN", params.stellarMass);
    std::snprintf(buffer, sizeof(buffer), "%.1f km", gravitationalRadiusKm(params.stellarMass));
    readoutRow(scaleLabel, buffer, kTextDim);

    y += 12.0f;
    divider(x, width, y);
    y += 10.0f;

    // ---- disk --------------------------------------------------------------
    sectionTitle("ACCRETION DISK", x, y);
    y += 24.0f;

    {
        Rectangle hit = {x, y - 2.0f, width, 24.0f};
        const bool hovering = CheckCollisionPointRec(mouse, hit);
        if (hovering) DrawRectangleRounded({x - 6, y - 4, width + 12, 26}, 0.35f, 4, kHover);
        const Rectangle box = {x, y + 3.0f, 11.0f, 11.0f};
        if (params.diskVisible) {
            DrawRectangle(static_cast<int>(box.x), static_cast<int>(box.y),
                          static_cast<int>(box.width), static_cast<int>(box.height), kAccent);
        }
        DrawRectangleLinesEx(box, 1.0f, params.diskVisible ? kAccent : kTextFaint);
        text("DISK VISIBLE", x + 20.0f, y, 10.0f, params.diskVisible ? kText : kTextDim);
        textRight(params.diskVisible ? "ON" : "OFF", right, y, 10.0f, kTextFaint);
        if (hovering && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            params.diskVisible = !params.diskVisible;
        }
        y += 26.0f;
    }

    const auto slider = [&](const char* label, float& value, float minimum, float maximum,
                            const char* format, const char* suffix) {
        const unsigned int id = idOf(label);
        Rectangle hit = {x, y - 3.0f, width, kRowHeight - 4.0f};
        const bool hovering = CheckCollisionPointRec(mouse, hit);
        const bool active = (m_activeSlider == id);

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            if (hovering && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) m_activeSlider = id;
            if (m_activeSlider == id) {
                const float t = std::clamp((mouse.x - x) / width, 0.0f, 1.0f);
                value = minimum + t * (maximum - minimum);
            }
        }

        if (hovering || active) {
            DrawRectangleRounded({x - 6, y - 5, width + 12, kRowHeight - 2}, 0.35f, 4, kHover);
        }

        std::snprintf(buffer, sizeof(buffer), format, static_cast<double>(value));
        text(label, x, y, 10.0f, kTextDim, 1.1f);
        std::string valueText = buffer;
        if (suffix != nullptr) valueText += suffix;
        textRight(valueText.c_str(), right, y, 10.0f, kText);

        const float trackY = y + kSliderTrackOffset;
        DrawRectangle(static_cast<int>(x), static_cast<int>(trackY), static_cast<int>(width), 1,
                      kTrack);
        const float t = (value - minimum) / (maximum - minimum);
        const float knobX = x + std::clamp(t, 0.0f, 1.0f) * width;
        DrawRectangle(static_cast<int>(x), static_cast<int>(trackY),
                      static_cast<int>(knobX - x), 1, kAccent);
        DrawCircleV({knobX, trackY + 0.5f}, (active || hovering) ? 3.6f : 3.0f,
                    (active || hovering) ? kAccent : Color{214, 222, 232, 255});
        y += kRowHeight;
    };

    slider("ROTATION SPEED", params.rotationSpeed, 0.0f, 3.0f, "%.2f", "x");
    slider("TEMPERATURE", params.temperature, 3000.0f, 20000.0f, "%.0f", " K");
    slider("OUTER RADIUS", params.outerRadius, 8.0f, 45.0f, "%.0f", " r_g");
    y += 10.0f;
    divider(x, width, y);
    y += 10.0f;

    // ---- render ------------------------------------------------------------
    sectionTitle("RENDER", x, y);
    y += 24.0f;

    slider("EXPOSURE", params.exposure, 0.1f, 4.0f, "%.2f", nullptr);
    slider("BLOOM", params.bloom, 0.0f, 2.0f, "%.2f", nullptr);

    {
        const float cellGap = 4.0f;
        const float cellWidth = (width - 2.0f * cellGap) / 3.0f;
        for (int i = 0; i < kQualityCount; ++i) {
            Rectangle cell = {x + i * (cellWidth + cellGap), y + 2.0f, cellWidth, 22.0f};
            const bool selected = static_cast<int>(params.quality) == i;
            const bool hovering = CheckCollisionPointRec(mouse, cell);
            if (selected) {
                DrawRectangleRec(cell, kAccentSoft);
                DrawRectangleLinesEx(cell, 1.0f, kAccent);
            } else {
                DrawRectangleLinesEx(cell, 1.0f, hovering ? Color{255, 255, 255, 80} : kTrack);
            }
            const char* label = qualitySettings(static_cast<QualityPreset>(i)).label;
            const float labelWidth = textWidth(label, 10.0f, 1.0f);
            text(label, cell.x + (cell.width - labelWidth) * 0.5f, cell.y + 6.0f, 10.0f,
                 selected ? kText : kTextDim, 1.0f);
            if (hovering && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                params.quality = static_cast<QualityPreset>(i);
            }
        }
        y += 30.0f;
    }

    y += 10.0f;
    divider(x, width, y);
    y += 10.0f;

    // ---- physical scale (mass changes units only, never the image) ---------
    if (layout.showScale) {
        sectionTitle("PHYSICAL SCALE", x, y);
        y += 24.0f;
        slider("MASS (SCALE ONLY)", params.stellarMass, 1.0f, 100.0f, "%.1f", " M_sun");

        std::snprintf(buffer, sizeof(buffer), "%.1f km",
                      gravitationalRadiusKm(params.stellarMass));
        text("r_g", x, y, 10.0f, kTextDim, 1.1f);
        textRight(buffer, right, y, 10.0f, kText);
        y += 17.0f;

        const double seconds = lightCrossingSeconds(params.stellarMass);
        if (seconds < 1e-3) {
            std::snprintf(buffer, sizeof(buffer), "%.1f us", seconds * 1e6);
        } else if (seconds < 1.0) {
            std::snprintf(buffer, sizeof(buffer), "%.2f ms", seconds * 1e3);
        } else {
            std::snprintf(buffer, sizeof(buffer), "%.2f s", seconds);
        }
        text("t_g = r_g / c", x, y, 10.0f, kTextDim, 1.1f);
        textRight(buffer, right, y, 10.0f, kText);
        y += 17.0f;
        y += 10.0f;
        divider(x, width, y);
        y += 10.0f;
    }

    // ---- legend ------------------------------------------------------------
    if (layout.showLegend) {
        sectionTitle("GEOMETRY", x, y);
        y += 24.0f;
        const auto legendRow = [&](Color color, const char* label, const char* value) {
            DrawCircleLines(static_cast<int>(x + 4), static_cast<int>(y + 5), 4.0f, color);
            text(label, x + 16.0f, y, 10.0f, kTextDim, 1.0f);
            textRight(value, right, y, 10.0f, kTextFaint);
            y += 17.0f;
        };
        legendRow({120, 130, 142, 255}, "EVENT HORIZON", "2 r_g");
        legendRow({150, 208, 255, 255}, "PHOTON SPHERE", "3 r_g");
        legendRow(kAccent, "INNERMOST ORBIT", "6 r_g");
    }

    // ---- control guide -----------------------------------------------------
    if (GetScreenHeight() > 620 && GetScreenWidth() > 760) {
        const char* line1 = "DRAG ORBIT   WHEEL ZOOM   MMB ELEVATION";
        const char* line2 = "SPACE PAUSE   R RESET   H UI   ESC EXIT";
        const float size = 10.0f;
        const float w1 = textWidth(line1, size, 1.2f);
        const float w2 = textWidth(line2, size, 1.2f);
        const float baseY = static_cast<float>(GetScreenHeight()) - 46.0f;
        text(line1, static_cast<float>(GetScreenWidth()) - 26.0f - w1, baseY, size, kTextFaint,
             1.2f);
        text(line2, static_cast<float>(GetScreenWidth()) - 26.0f - w2, baseY + 15.0f, size,
             kTextFaint, 1.2f);
    }
}

}  // namespace bh
