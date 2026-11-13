#include "stm32_fdcan_phy.hpp"

// Only compile HAL calls when built inside an STM32 HAL project with FDCAN enabled
#if defined(HAL_FDCAN_MODULE_ENABLED)

#include <cstring>
#include "main.h"

namespace solrac
{
    namespace
    {

        constexpr std::uint8_t kDlcToBytes[16] = {
            0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

        std::uint32_t bytesToHalDlc(const std::uint8_t len) noexcept
        {
            if (len <= 8U)
            {
                return static_cast<std::uint32_t>(len) << 16U;
            }
            if (len <= 12U)
            {
                return FDCAN_DLC_BYTES_12;
            }
            if (len <= 16U)
            {
                return FDCAN_DLC_BYTES_16;
            }
            if (len <= 20U)
            {
                return FDCAN_DLC_BYTES_20;
            }
            if (len <= 24U)
            {
                return FDCAN_DLC_BYTES_24;
            }
            if (len <= 32U)
            {
                return FDCAN_DLC_BYTES_32;
            }
            if (len <= 48U)
            {
                return FDCAN_DLC_BYTES_48;
            }
            return FDCAN_DLC_BYTES_64;
        }

        std::uint8_t halDlcToBytes(const std::uint32_t dlc_field) noexcept
        {
            const std::uint8_t idx = static_cast<std::uint8_t>((dlc_field >> 16U) & 0x0FU);
            return kDlcToBytes[idx];
        }

    } // namespace

    Stm32FdcanPhy::Stm32FdcanPhy(FDCAN_HandleTypeDef &hfdcan) noexcept : hfdcan_(hfdcan) {}

    bool Stm32FdcanPhy::init() noexcept
    {
        // Accept all 29-bit extended frames into RX FIFO 0; SOLRAC filters by subscription
        FDCAN_FilterTypeDef filter{};
        filter.IdType = FDCAN_EXTENDED_ID;
        filter.FilterIndex = 0U;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = 0x00000000U;
        filter.FilterID2 = 0x00000000U;

        if (HAL_FDCAN_ConfigFilter(&hfdcan_, &filter) != HAL_OK)
        {
            return false;
        }
        if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan_,
                                         FDCAN_REJECT,
                                         FDCAN_ACCEPT_IN_RX_FIFO0,
                                         FDCAN_FILTER_REMOTE,
                                         FDCAN_FILTER_REMOTE) != HAL_OK)
        {
            return false;
        }
        if (HAL_FDCAN_Start(&hfdcan_) != HAL_OK)
        {
            return false;
        }

        started_ = true;
        return true;
    }

    bool Stm32FdcanPhy::transmit(const CanFdFrame &frame) noexcept
    {
        if (!started_ || (frame.payload_size > kCanFdMaxPayloadBytes))
        {
            return false;
        }
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan_) == 0U)
        {
            return false;
        }

        FDCAN_TxHeaderTypeDef tx_hdr{};
        tx_hdr.Identifier = frame.extended_id & 0x1FFFFFFFU;
        tx_hdr.IdType = FDCAN_EXTENDED_ID;
        tx_hdr.TxFrameType = FDCAN_DATA_FRAME;
        tx_hdr.DataLength = bytesToHalDlc(frame.payload_size);
        tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_hdr.BitRateSwitch = FDCAN_BRS_ON;
        tx_hdr.FDFormat = FDCAN_FD_CAN;
        tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_hdr.MessageMarker = 0U;

        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan_, &tx_hdr, frame.payload) == HAL_OK;
    }

    bool Stm32FdcanPhy::receive(CanFdFrame &out_frame) noexcept
    {
        if (!started_)
        {
            return false;
        }
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan_, FDCAN_RX_FIFO0) == 0U)
        {
            return false;
        }

        FDCAN_RxHeaderTypeDef rx_hdr{};
        std::uint8_t rx_buf[kCanFdMaxPayloadBytes]{};

        if (HAL_FDCAN_GetRxMessage(&hfdcan_, FDCAN_RX_FIFO0, &rx_hdr, rx_buf) != HAL_OK)
        {
            return false;
        }
        if (rx_hdr.IdType != FDCAN_EXTENDED_ID)
        {
            return false;
        }

        out_frame.extended_id = rx_hdr.Identifier & 0x1FFFFFFFU;
        out_frame.payload_size = halDlcToBytes(rx_hdr.DataLength);
        std::memcpy(out_frame.payload, rx_buf, out_frame.payload_size);
        return true;
    }

    bool Stm32FdcanPhy::isHealthy() const noexcept
    {
        if (!started_)
        {
            return false;
        }
        FDCAN_ProtocolStatusTypeDef status{};
        if (HAL_FDCAN_GetProtocolStatus(&hfdcan_, &status) != HAL_OK)
        {
            return false;
        }
        return (status.BusOff == 0U) && (status.ErrorPassive == 0U);
    }

} // namespace solrac

#endif // HAL_FDCAN_MODULE_ENABLED