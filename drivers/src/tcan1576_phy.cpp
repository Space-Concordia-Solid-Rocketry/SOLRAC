#include "tcan1576_phy.hpp"

namespace solrac
{
    namespace
    {

        // TCAN1576-Q1 / TCAN1145-Q1 SPI Register Map
        constexpr std::uint8_t kRegModeCntrl = 0x10U; // Mode control & SW_EN
        constexpr std::uint8_t kRegIntGlobal = 0x50U; // Global interrupt status
        constexpr std::uint8_t kRegInt1 = 0x51U;      // Interrupt bank 1 (W1C)
        constexpr std::uint8_t kRegInt2 = 0x52U;      // Interrupt bank 2: POR / UVSUP (W1C)
        constexpr std::uint8_t kRegInt3 = 0x53U;      // Interrupt bank 3 (W1C)
        constexpr std::uint8_t kRegIntCanBus = 0x54U; // CAN bus fault interrupts (W1C)

        constexpr std::uint8_t kModeMask = 0x07U;    // MODE_SEL bits [2:0]
        constexpr std::uint8_t kSwEnBit = 0x80U;     // Selective Wake Enable bit [7]
        constexpr std::uint8_t kSpiWriteBit = 0x01U; // Byte 0 LSB = 1 for write, 0 for read

    } // namespace

    Tcan1576Phy::Tcan1576Phy(ICanPhy &controller,
                             ITcan1576Spi &spi,
                             const bool enable_partial_nw) noexcept
        : controller_(controller), spi_(spi), enable_partial_nw_(enable_partial_nw) {}

    bool Tcan1576Phy::readRegister(const std::uint8_t reg_addr, std::uint8_t &out_val) const noexcept
    {
        const std::uint8_t cmd_byte = static_cast<std::uint8_t>((reg_addr & 0x7FU) << 1U);
        const std::uint16_t tx_word =
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(cmd_byte) << 8U);
        std::uint16_t rx_word = 0U;

        if (!spi_.transfer16(tx_word, rx_word))
        {
            return false;
        }
        out_val = static_cast<std::uint8_t>(rx_word & 0x00FFU);
        return true;
    }

    bool Tcan1576Phy::writeRegister(const std::uint8_t reg_addr, const std::uint8_t value) noexcept
    {
        const std::uint8_t cmd_byte =
            static_cast<std::uint8_t>(((reg_addr & 0x7FU) << 1U) | kSpiWriteBit);
        const std::uint16_t tx_word =
            static_cast<std::uint16_t>((static_cast<std::uint16_t>(cmd_byte) << 8U) | value);
        std::uint16_t rx_word = 0U;

        return spi_.transfer16(tx_word, rx_word);
    }

    bool Tcan1576Phy::clearFaultsAndInterrupts(std::uint8_t &out_global_int) noexcept
    {
        // Write-1-to-clear startup POR, undervoltage, and bus fault interrupt latches
        if (!writeRegister(kRegInt1, 0xFFU) ||
            !writeRegister(kRegInt2, 0xFFU) ||
            !writeRegister(kRegInt3, 0xFFU) ||
            !writeRegister(kRegIntCanBus, 0xFFU))
        {
            return false;
        }
        return readRegister(kRegIntGlobal, out_global_int);
    }

    bool Tcan1576Phy::setMode(const Tcan1576Mode mode) noexcept
    {
        const std::uint8_t sw_flag = enable_partial_nw_ ? kSwEnBit : 0x00U;
        const std::uint8_t target_reg =
            static_cast<std::uint8_t>(sw_flag | (static_cast<std::uint8_t>(mode) & kModeMask));

        if (!writeRegister(kRegModeCntrl, target_reg))
        {
            return false;
        }

        // In Sleep mode, INH drops immediately and MCU power is cut; skip readback
        if (mode == Tcan1576Mode::kSleep)
        {
            current_mode_ = Tcan1576Mode::kSleep;
            return true;
        }

        // Verify mode transition over SPI
        std::uint8_t readback = 0U;
        if (!readRegister(kRegModeCntrl, readback))
        {
            return false;
        }
        if ((readback & kModeMask) != (static_cast<std::uint8_t>(mode) & kModeMask))
        {
            return false;
        }

        current_mode_ = mode;
        return true;
    }

    bool Tcan1576Phy::init() noexcept
    {
        std::uint8_t global_int = 0U;
        if (!clearFaultsAndInterrupts(global_int))
        {
            return false;
        }
        if (!controller_.init())
        {
            return false;
        }
        if (!setMode(Tcan1576Mode::kNormal))
        {
            return false;
        }

        initialized_ = true;
        return true;
    }

    bool Tcan1576Phy::enterPowerDownSleep() noexcept
    {
        // Ensure wake interrupt flags are clear so the transceiver can wake cleanly later
        std::uint8_t ignored = 0U;
        (void)clearFaultsAndInterrupts(ignored);
        return setMode(Tcan1576Mode::kSleep);
    }

    bool Tcan1576Phy::transmit(const CanFdFrame &frame) noexcept
    {
        if (!initialized_ || (current_mode_ != Tcan1576Mode::kNormal))
        {
            return false;
        }
        return controller_.transmit(frame);
    }

    bool Tcan1576Phy::receive(CanFdFrame &out_frame) noexcept
    {
        if (!initialized_)
        {
            return false;
        }
        if ((current_mode_ != Tcan1576Mode::kNormal) &&
            (current_mode_ != Tcan1576Mode::kListenOnly))
        {
            return false;
        }
        return controller_.receive(out_frame);
    }

    bool Tcan1576Phy::isHealthy() const noexcept
    {
        if (!initialized_ || (current_mode_ != Tcan1576Mode::kNormal))
        {
            return false;
        }
        return controller_.isHealthy();
    }

} // namespace solrac