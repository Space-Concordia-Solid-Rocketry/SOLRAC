#ifndef SOLRAC_DRIVERS_TCAN1576_PHY_HPP
#define SOLRAC_DRIVERS_TCAN1576_PHY_HPP

#include <cstdint>
#include "can_phy.hpp"

namespace solrac
{

    /// @brief Operating modes for the TI TCAN1576DMTRQ1 CAN-FD SIC transceiver.
    enum class Tcan1576Mode : std::uint8_t
    {
        kSleep = 0x01U,      ///< Low-power sleep: INH pin driven LOW (cuts board regulator power)
        kStandby = 0x04U,    ///< Standby: INH pin HIGH, CAN driver disabled
        kListenOnly = 0x05U, ///< Listen-only: RX active, TX driver disabled
        kNormal = 0x07U      ///< Normal operation: INH pin HIGH, full CAN-FD TX/RX active
    };

    /// @brief Abstract 16-bit SPI Mode 0 (CPOL=0, CPHA=0, <= 4 MHz) transport for TCAN1576.
    ///
    /// @details
    /// RFC 2119 Requirements:
    /// - Implementations MUST assert nCS LOW before clocking 16 bits and deassert nCS HIGH after.
    /// - Implementations MUST NOT throw exceptions or allocate dynamic memory.
    class ITcan1576Spi
    {
    public:
        virtual ~ITcan1576Spi() noexcept = default;

        /// @brief Performs a full-duplex 16-bit SPI transfer with the TCAN1576.
        /// @param[in]  tx_word 16-bit command word: [15:9]=addr, [8]=R/W (1=Write, 0=Read), [7:0]=data.
        /// @param[out] rx_word 16-bit response word: [15:8]=INT_GLOBAL, [7:0]=register value.
        /// @return true if SPI transaction succeeded; false on SPI bus timeout/error.
        [[nodiscard]] virtual bool transfer16(std::uint16_t tx_word,
                                              std::uint16_t &rx_word) noexcept = 0;

    protected:
        ITcan1576Spi() noexcept = default;
        ITcan1576Spi(const ITcan1576Spi &) = delete;
        ITcan1576Spi &operator=(const ITcan1576Spi &) = delete;
    };

    /// @brief Physical layer wrapper pairing an STM32 FDCAN controller with a TCAN1576DMTRQ1 transceiver.
    ///
    /// @details
    /// RFC 2119 Operational Specification:
    /// - init() SHALL clear startup interrupt flags, initialize the underlying CAN controller,
    ///   and place the TCAN1576 into Tcan1576Mode::kNormal.
    /// - On power-gated peripheral boards, enterPowerDownSleep() SHALL command Tcan1576Mode::kSleep,
    ///   driving the INH pin LOW to disable downstream board power rails.
    class Tcan1576Phy final : public ICanPhy
    {
    public:
        /// @param[in] controller        Underlying CAN-FD controller (e.g., Stm32FdcanPhy).
        /// @param[in] spi               Board-specific 16-bit SPI interface to the TCAN1576.
        /// @param[in] enable_partial_nw Set true on power-gated boards using Selective Wake (SW_EN).
        Tcan1576Phy(ICanPhy &controller,
                    ITcan1576Spi &spi,
                    bool enable_partial_nw = false) noexcept;

        ~Tcan1576Phy() noexcept override = default;

        Tcan1576Phy(const Tcan1576Phy &) = delete;
        Tcan1576Phy &operator=(const Tcan1576Phy &) = delete;

        /// @brief Initializes controller + TCAN1576, clears POR flags, and enters Normal mode.
        [[nodiscard]] bool init() noexcept override;

        [[nodiscard]] bool transmit(const CanFdFrame &frame) noexcept override;
        [[nodiscard]] bool receive(CanFdFrame &out_frame) noexcept override;
        [[nodiscard]] bool isHealthy() const noexcept override;

        /// @brief Switches transceiver operating mode (e.g., kNormal, kListenOnly, kStandby, kSleep).
        [[nodiscard]] bool setMode(Tcan1576Mode mode) noexcept;

        /// @brief Cuts peripheral board power by commanding Sleep mode (drives INH pin LOW).
        /// @details WARNING: Once executed, the host MCU loses regulator power until a CAN wake event.
        [[nodiscard]] bool enterPowerDownSleep() noexcept;

        /// @brief Clears latched interrupt registers (0x51..0x54) and returns INT_GLOBAL (0x50).
        [[nodiscard]] bool clearFaultsAndInterrupts(std::uint8_t &out_global_int) noexcept;

        /// @brief Reads a single 8-bit TCAN1576 SPI register.
        [[nodiscard]] bool readRegister(std::uint8_t reg_addr, std::uint8_t &out_val) const noexcept;

        /// @brief Writes a single 8-bit TCAN1576 SPI register.
        [[nodiscard]] bool writeRegister(std::uint8_t reg_addr, std::uint8_t value) noexcept;

    private:
        ICanPhy &controller_;
        ITcan1576Spi &spi_;
        bool enable_partial_nw_;
        Tcan1576Mode current_mode_{Tcan1576Mode::kStandby};
        bool initialized_{false};
    };

} // namespace solrac

#endif // SOLRAC_DRIVERS_TCAN1576_PHY_HPP