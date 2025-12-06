/**
 * @file ws2812.cpp
 *
 * @brief CAuDri - WS2812 LED driver
 *
 * This driver implements control for WS2812 (NeoPixel) addressable RGB LEDs. I can be templated to support an arbitrary
 * number of LEDs in a strip.
 *
 * The driver uses a timer peripheral and DMA stream to generate the timings required by the WS2812 protocol.
 */
#include "ws2812.hpp"

#include "logger.h"

static constexpr uint32_t THREAD_TRANSMIT_TIMEOUT_MS = 10;

static constexpr uint32_t TX_PERIOD_FULL_NS = 1250;  // Full bit period in nanoseconds
static constexpr uint32_t TX_HIGH_TIME_0_NS = 350;   // High time for a '0' bit in nanoseconds
static constexpr uint32_t TX_HIGH_TIME_1_NS = 900;   // High time for a '1' bit in nanoseconds

static constexpr uint32_t TIMER_RESOLUTION_NS = 50;  // Timer resolution in nanoseconds (20 MHz timer clock)
static constexpr uint32_t TIMER_ARR_VALUE = (TX_PERIOD_FULL_NS / TIMER_RESOLUTION_NS) - 1;  // Timer auto-reload value for 1.25us period

static constexpr uint32_t TRANSMIT_COMPLETE_FLAG = 0x40;  // Thread flag indicating DMA transmit complete (change if conflicts arise)

WS2812Driver::WS2812Driver(const char* name) : Driver(name) {}

WS2812Driver::WS2812Driver(const char* name, const Config& config, const DMABuffer& dma_buffer) : Driver(name) {
    init(config, dma_buffer);
}

WS2812Driver::~WS2812Driver() = default;

bool WS2812Driver::init(const Config& config, const DMABuffer& dma_buffer) {
    if (state != State::UNINITIALIZED) {
        return false;
    }

    this->config = &config;
    this->dma_buffer = dma_buffer;
    this->led_count = dma_buffer.led_count;

    // Check for valid configuration
    if (config.htim == nullptr || config.hdma == nullptr) {
        LogError("WS2812: Invalid timer or DMA handle");
        setState(State::ERROR);
        return false;
    }

    // Check for valid DMA buffer configuration
    if (dma_buffer.buffer == nullptr || dma_buffer.size == 0 || dma_buffer.led_count == 0) {
        LogError("WS2812: Invalid DMA buffer configuration");
        setState(State::ERROR);
        return false;
    }

    // Make sure the DMA buffer is large enough for the provided number of LEDs
    size_t expected_size = dma_buffer.led_count * WS2812_BITS_PER_LED + WS2812_RESET_TICKS;
    if (dma_buffer.timer_word_size == DMABuffer::TimerWordSize::SIZE_16BIT) {
        expected_size *= sizeof(uint16_t);
    } else if (dma_buffer.timer_word_size == DMABuffer::TimerWordSize::SIZE_32BIT) {
        expected_size *= sizeof(uint32_t);
    } else {
        LogError("WS2812: Invalid timer word size");
        setState(State::ERROR);
        return false;
    }

    if (dma_buffer.size != expected_size) {
        LogError("WS2812: DMA buffer size (%zu bytes) does not match expected size (%zu bytes) for %lu LEDs",
                 dma_buffer.size,
                 expected_size,
                 dma_buffer.led_count);
        setState(State::ERROR);
        return false;
    }

    // Fill the DMA buffer with zeroes for the reset period
    if (dma_buffer.timer_word_size == DMABuffer::TimerWordSize::SIZE_16BIT) {
        auto* buffer = static_cast<uint16_t*>(dma_buffer.buffer);
        for (size_t i = 0; i < (dma_buffer.size / sizeof(uint16_t)) - 1; ++i) {
            buffer[i] = 0;
        }
    } else if (dma_buffer.timer_word_size == DMABuffer::TimerWordSize::SIZE_32BIT) {
        auto* buffer = static_cast<uint32_t*>(dma_buffer.buffer);
        for (size_t i = 0; i < (dma_buffer.size / sizeof(uint32_t)) - 1; ++i) {
            buffer[i] = 0;
        }
    }

    if (!initTimer()) {
        LogError("WS2812: Failed to initialize timer");
        setState(State::ERROR);
        return false;
    }

    if (!initDMA()) {
        LogError("WS2812: Failed to initialize DMA");
        setState(State::ERROR);
        return false;
    }

    // Initialize mutex for thread safe access
    snprintf(mutex_name_buffer, sizeof(mutex_name_buffer), "%s Mutex", getName());
    mutex_attributes.name = mutex_name_buffer;
    mutex_attributes.attr_bits = osMutexRecursive;
    mutex_attributes.cb_mem = &mutex_control_block;
    mutex_attributes.cb_size = sizeof(mutex_control_block);
    transmit_mutex = osMutexNew(&mutex_attributes);
    if (transmit_mutex == nullptr) {
        LogError("WS2812: Failed to create driver mutex");
        setState(State::ERROR);
        return false;
    }

    setState(State::RUNNING);

    // Turn off all LEDs initially
    if (!disableAll()) {
        LogError("WS2812: Failed to disable all LEDs");
        setState(State::ERROR);
        return false;
    }

    LogInfo("WS2812: Driver initialized for %lu LEDs", dma_buffer.led_count);
    return true;
}

uint32_t WS2812Driver::getLEDCount() const { return led_count; }

bool WS2812Driver::setColor(Color color) {
    if (state != State::RUNNING) {
        return false;
    }
    if (osMutexAcquire(transmit_mutex, THREAD_TRANSMIT_TIMEOUT_MS) != osOK) {
        LogWarning("WS2812: Timeout acquiring transmit mutex");
        return false;
    }
    for (uint32_t i = 0; i < led_count; ++i) {
        if (!writeToBuffer(i, color)) {
            LogDebug("WS2812: Failed to write color to buffer at index %lu", i);
        }
    }
    auto result = transmitBuffer();
    osMutexRelease(transmit_mutex);
    return result;
}

bool WS2812Driver::setColor(uint32_t index, Color color) {
    if (state != State::RUNNING) {
        return false;
    }
    if (index >= led_count) {
        LogWarning("WS2812: LED index %lu out of bounds (max %lu)", index, led_count - 1);
        return false;
    }
    if (osMutexAcquire(transmit_mutex, THREAD_TRANSMIT_TIMEOUT_MS) != osOK) {
        LogWarning("WS2812: Timeout acquiring transmit mutex");
        return false;
    }
    if (!writeToBuffer(index, color)) {
        LogDebug("WS2812: Failed to write color to buffer at index %lu", index);
        osMutexRelease(transmit_mutex);
        return false;
    }
    auto result = transmitBuffer();
    osMutexRelease(transmit_mutex);
    return result;
}

bool WS2812Driver::setColors(const Color* colors, uint32_t start_index, uint32_t count) {
    if (state != State::RUNNING) {
        return false;
    }
    if (start_index + count > led_count) {
        LogWarning("WS2812: LED range (%lu to %lu) out of bounds (max %lu)", start_index, start_index + count - 1, led_count - 1);
        return false;
    }
    if (osMutexAcquire(transmit_mutex, THREAD_TRANSMIT_TIMEOUT_MS) != osOK) {
        LogWarning("WS2812: Timeout acquiring transmit mutex");
        return false;
    }
    for (uint32_t i = 0; i < count; ++i) {
        if (!writeToBuffer(start_index + i, colors[i])) {
            LogDebug("WS2812: Failed to write color to buffer at index %lu", start_index + i);
        }
    }
    auto result = transmitBuffer();
    osMutexRelease(transmit_mutex);
    return result;
}

bool WS2812Driver::disableAll() { return setColor(Color(0, 0, 0)); }

bool WS2812Driver::initTimer() {
    uint32_t timer_clock_freq = 0;
    TIM_HandleTypeDef* htim = config->htim;

    // Timers can be on different APB buses and thus have different clock frequencies
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 || htim->Instance == TIM10 ||
        htim->Instance == TIM11) {
        // TIM1, TIM8, TIM9, TIM10, TIM11 are on APB2
        // This might only be valid for STM32F4xx and needs to be adapted for other STM32 series
        timer_clock_freq = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
            // Timer clock is automatically doubled if APB prescaler > 1
            timer_clock_freq *= 2;
        }
    } else {
        // All other timers are on APB1
        timer_clock_freq = HAL_RCC_GetPCLK1Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
            timer_clock_freq *= 2;
        }
    }

    // Scale the timer clock frequency for the desired resolution
    uint32_t prescaler = (timer_clock_freq / (1e9 / TIMER_RESOLUTION_NS)) - 1;
    if (prescaler > 0xFFFF || prescaler == 0) {
        LogError("WS2812: Invalid prescaler value, prescaler = %lu", prescaler);
        return false;
    }

    // One timer clock tick equals 50 ns
    __HAL_TIM_SET_PRESCALER(htim, prescaler);
    __HAL_TIM_SET_AUTORELOAD(htim, TIMER_ARR_VALUE);
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_SET_COMPARE(htim, config->tim_channel, 0);

    LogDebug("WS2812: Timer initialized on %s, prescaler = %lu, ARR = %lu",
             (htim->Instance == TIM1 || htim->Instance == TIM8) ? "APB2" : "APB1",
             prescaler,
             TIMER_ARR_VALUE);

    return HAL_TIM_PWM_Start(htim, config->tim_channel) == HAL_OK;
}

bool WS2812Driver::initDMA() {
    // We assume the DMA is already initialized by CubeMX and will only confirm
    uint32_t dma_id = 0xFFFFFFFF;
    switch (config->tim_channel) {
        case TIM_CHANNEL_1:
            dma_id = TIM_DMA_ID_CC1;
            break;
        case TIM_CHANNEL_2:
            dma_id = TIM_DMA_ID_CC2;
            break;
        case TIM_CHANNEL_3:
            dma_id = TIM_DMA_ID_CC3;
            break;
        case TIM_CHANNEL_4:
            dma_id = TIM_DMA_ID_CC4;
            break;
        default:
            LogError("WS2812: Unsupported timer channel");
            return false;
    }

    if (dma_id > TIM_DMA_ID_TRIGGER || config->htim->hdma[dma_id] != config->hdma) {
        LogError("WS2812: DMA is not linked to the timer channel");
        return false;
    }

    // Store the driver instance in the DMA's parent pointer for callback access
    config->hdma->Parent = this;

    auto status = HAL_DMA_RegisterCallback(config->hdma, HAL_DMA_XFER_CPLT_CB_ID, dmaCompleteCallback);
    if (status != HAL_OK) {
        LogError("WS2812: Failed to register DMA complete callback");
        return false;
    }

    status = HAL_DMA_RegisterCallback(config->hdma, HAL_DMA_XFER_ERROR_CB_ID, dmaErrorCallback);
    if (status != HAL_OK) {
        LogError("WS2812: Failed to register DMA error callback");
        return false;
    }

    LogDebug("WS2812: DMA initialized for timer channel %lu", config->tim_channel);
    return true;
}

bool WS2812Driver::writeToBuffer(uint32_t index, Color color) {
    if (index >= led_count) {
        LogWarning("WS2812: LED index %lu out of bounds (max %lu)", index, led_count - 1);
        return false;
    }

    // The actual PWM timings will be stored in the DMA buffer for each color bit
    const uint16_t ticks_0 = TX_HIGH_TIME_0_NS / TIMER_RESOLUTION_NS;
    const uint16_t ticks_1 = TX_HIGH_TIME_1_NS / TIMER_RESOLUTION_NS;
    uint8_t colors[3] = {color.green, color.red, color.blue};  // WS2812 expects GRB order

    for (uint32_t color_channel = 0; color_channel < 3; ++color_channel) {
        for (uint8_t bit = 0; bit < 8; ++bit) {
            uint32_t bit_index = (colors[color_channel] >> (7u - bit)) & 0x01;  // Extract the bit value (MSB first)
            size_t buffer_index = (index * WS2812_BITS_PER_LED) + (color_channel * 8) + bit;

            // Depending on the timer word size, write the appropriate value to the DMA buffer
            if (dma_buffer.timer_word_size == DMABuffer::TimerWordSize::SIZE_16BIT) {
                auto* buffer = static_cast<uint16_t*>(dma_buffer.buffer);
                buffer[buffer_index] = (bit_index == 1) ? ticks_1 : ticks_0;
            } else if (dma_buffer.timer_word_size == DMABuffer::TimerWordSize::SIZE_32BIT) {
                auto* buffer = static_cast<uint32_t*>(dma_buffer.buffer);
                buffer[buffer_index] = (bit_index == 1) ? ticks_1 : ticks_0;
            } else {
                LogError("WS2812: Invalid timer word size");
                return false;
            }
        }
    }
    return true;
}

bool WS2812Driver::transmitBuffer() {
    if (state != State::RUNNING) {
        return false;
    }

    uint32_t* timer_ccr = nullptr;
    switch (config->tim_channel) {
        case TIM_CHANNEL_1:
            __HAL_TIM_ENABLE_DMA(config->htim, TIM_DMA_CC1);
            timer_ccr = (uint32_t*)(&config->htim->Instance->CCR1);
            break;
        case TIM_CHANNEL_2:
            __HAL_TIM_ENABLE_DMA(config->htim, TIM_DMA_CC2);
            timer_ccr = (uint32_t*)(&config->htim->Instance->CCR2);
            break;
        case TIM_CHANNEL_3:
            __HAL_TIM_ENABLE_DMA(config->htim, TIM_DMA_CC3);
            timer_ccr = (uint32_t*)(&config->htim->Instance->CCR3);
            break;
        case TIM_CHANNEL_4:
            __HAL_TIM_ENABLE_DMA(config->htim, TIM_DMA_CC4);
            timer_ccr = (uint32_t*)(&config->htim->Instance->CCR4);
            break;
        default:
            LogError("WS2812: Unsupported timer channel");
            return false;
    }

    osThreadFlagsClear(TRANSMIT_COMPLETE_FLAG);
    dma_error_raised = false;

    // Store the current thread ID to notify upon DMA completion
    transmit_thread_id = osThreadGetId();

    // Start the DMA transfer in interrupt mode
    uint32_t data_length = dma_buffer.size;
    data_length /= (dma_buffer.timer_word_size == DMABuffer::TimerWordSize::SIZE_16BIT) ? sizeof(uint16_t) : sizeof(uint32_t);

    auto status = HAL_DMA_Start_IT(
        config->hdma, reinterpret_cast<uint32_t>(dma_buffer.buffer), reinterpret_cast<uint32_t>(timer_ccr), data_length);
    if (status != HAL_OK) {
        LogError("WS2812: Failed to start DMA transfer");
        transmit_thread_id = nullptr;
        return false;
    }

    auto flags = osThreadFlagsWait(TRANSMIT_COMPLETE_FLAG, osFlagsWaitAny, THREAD_TRANSMIT_TIMEOUT_MS);
    if (flags == osFlagsErrorTimeout) {
        LogWarning("WS2812: Timeout waiting for DMA transmit to complete");
        transmit_thread_id = nullptr;
        return false;
    } else if (!(flags & TRANSMIT_COMPLETE_FLAG)) {
        LogWarning("WS2812: Unexpected thread flags received: 0x%08lX", flags);
        transmit_thread_id = nullptr;
        return false;
    }

    if (dma_error_raised) {
        dma_error_raised = false;
        LogWarning("WS2812: DMA transfer completed with error");
    }

    transmit_thread_id = nullptr;
    if (dma_error_raised) {
        LogWarning("WS2812: DMA transfer error occurred");
        return false;
    }
    return true;
}

void WS2812Driver::onDMAComplete() {
    if (transmit_thread_id != nullptr) {
        osThreadFlagsSet(transmit_thread_id, TRANSMIT_COMPLETE_FLAG);
    }
}

void WS2812Driver::onDMAError() {
    dma_error_raised = true;
    if (transmit_thread_id != nullptr) {
        osThreadFlagsSet(transmit_thread_id, TRANSMIT_COMPLETE_FLAG);
    }
}

void WS2812Driver::dmaCompleteCallback(DMA_HandleTypeDef* hdma) {
    if (hdma->Parent != nullptr) {
        auto* driver = static_cast<WS2812Driver*>(hdma->Parent);
        driver->onDMAComplete();
    }
}

void WS2812Driver::dmaErrorCallback(DMA_HandleTypeDef* hdma) {
    if (hdma->Parent != nullptr) {
        auto* driver = static_cast<WS2812Driver*>(hdma->Parent);
        driver->onDMAError();
    }
}