/**
 * @file ws2812.hpp
 *
 * @brief CAuDri - WS2812 LED driver
 *
 * This driver implements control for WS2812 (NeoPixel) addressable RGB LEDs. I can be templated to support an arbitrary
 * number of LEDs in a strip.
 *
 * The driver uses a timer peripheral and DMA stream to generate the timings required by the WS2812 protocol.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "callback_wrapper.hpp"
#include "cmsis_os2.h"
#include "colors.hpp"
#include "driver.hpp"
#include "stm32f4xx_hal.h"

constexpr uint32_t WS2812_BITS_PER_LED = 24;  // Number of bits per WS2812 LED (8 bits each for R, G, B)
constexpr uint32_t WS2812_RESET_TICKS = 200;   // Number of timer ticks for the reset signal (~250us)

/**
 * @brief Non-templated WS2812 driver class for runtime defined number of LEDs
 *
 * This class should not be used directly. Instead, use the templated WS2812 class that allows the use of a statically allocated DMA buffer.
 */
class WS2812Driver : public Driver {
   public:
    /**
     * @brief Configuration struct for the WS2812 driver
     *
     * @param htim Pointer to the initialized timer handle
     * @param tim_channel Timer channel used for generating the WS2812 signal
     * @param hdma Pointer to the initialized DMA handle associated with the timer channel
     */
    struct Config {
        TIM_HandleTypeDef* htim = nullptr;
        uint32_t tim_channel = 0;
        DMA_HandleTypeDef* hdma = nullptr;
    };

    /**
     * @brief Struct for passing the DMA buffer information
     *
     * Depending on which timer peripheral is used, a buffer with either 16-bit or 32-bit words must be used.
     * This is a hardware limitation with DMA transfers to half-word registers.
     *
     * @param timer_word_size Size of the timer register (16-bit or 32-bit)
     * @param buffer Pointer to the timing buffer
     * @param size Size of the timing buffer in bytes
     * @param led_count Number of LEDs in the strip
     */
    struct DMABuffer {
        enum class TimerWordSize { SIZE_16BIT, SIZE_32BIT } timer_word_size = TimerWordSize::SIZE_16BIT;
        void* buffer = nullptr;
        size_t size = 0;
        uint32_t led_count = 0;
    };

    WS2812Driver(const char* name);
    WS2812Driver(const char* name, const Config& config, const DMABuffer& dma_buffer);
    ~WS2812Driver() override;

    bool init(const Config& config, const DMABuffer& dma_buffer);

    uint32_t getLEDCount() const;

    bool setColor(Color color);
    bool setColor(uint32_t index, Color color);
    bool setColors(const Color* colors, uint32_t start_index, uint32_t count);
    bool disableAll();

   protected:
    const Config* config = nullptr;
    alignas(uint16_t) DMABuffer dma_buffer{};
    uint32_t led_count = 0;

    osMutexId_t transmit_mutex = nullptr;
    osMutexAttr_t mutex_attributes{};
    StaticSemaphore_t mutex_control_block{};
    char mutex_name_buffer[16]{};

    osThreadId_t transmit_thread_id = nullptr;
    bool dma_error_raised = false;

    bool initTimer();
    bool initDMA();
    bool writeToBuffer(uint32_t index, Color color);
    bool transmitBuffer();

    void onDMAComplete();
    void onDMAError();

    static void dmaCompleteCallback(DMA_HandleTypeDef* hdma);
    static void dmaErrorCallback(DMA_HandleTypeDef* hdma);
};

/**
 * @brief Templated WS2812 driver class for a fixed number of LEDs
 *
 * This class allows for a statically allocated DMA buffer.
 *
 * @tparam LEDCount Number of WS2812 LEDs in the strip
 * @tparam TimerWordType Type of the timer word (uint16_t or uint32_t) depending on the timer peripheral used
 */
template <uint32_t LEDCount, typename TimerWordType = uint16_t>
class WS2812 : public WS2812Driver {
   public:
    using Config = WS2812Driver::Config;

    WS2812(const char* name);
    WS2812(const char* name, const Config& config);
    ~WS2812() override;

    bool init(const Config& config);

   private:
    std::array<TimerWordType, (WS2812_BITS_PER_LED * LEDCount) + WS2812_RESET_TICKS> dma_buffer{};  // Color Data + Reset signal
};

template <uint32_t LEDCount, typename TimerWordType>
WS2812<LEDCount, TimerWordType>::WS2812(const char* name) : WS2812Driver(name) {}

template <uint32_t LEDCount, typename TimerWordType>
WS2812<LEDCount, TimerWordType>::WS2812(const char* name, const Config& config) : WS2812Driver(name) {
    init(config);
}

template <uint32_t LEDCount, typename TimerWordType>
WS2812<LEDCount, TimerWordType>::~WS2812() = default;

template <uint32_t LEDCount, typename TimerWordType>
bool WS2812<LEDCount, TimerWordType>::init(const Config& config) {
    // DMA buffer setup
    WS2812Driver::DMABuffer dma_buffer_handle{};
    dma_buffer_handle.buffer = dma_buffer.data();
    dma_buffer_handle.size = sizeof(dma_buffer);
    dma_buffer_handle.led_count = LEDCount;
    if constexpr (std::is_same_v<TimerWordType, uint16_t>) {
        dma_buffer_handle.timer_word_size = WS2812Driver::DMABuffer::TimerWordSize::SIZE_16BIT;
    } else if constexpr (std::is_same_v<TimerWordType, uint32_t>) {
        dma_buffer_handle.timer_word_size = WS2812Driver::DMABuffer::TimerWordSize::SIZE_32BIT;
    } else {
        LogError("WS2812: Unsupported TimerWordType");
        return false;
    }
    return WS2812Driver::init(config, dma_buffer_handle);
}