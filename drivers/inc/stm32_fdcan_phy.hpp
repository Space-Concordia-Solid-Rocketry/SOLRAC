#ifndef SOLRAC_DRIVERS_STM32_FDCAN_PHY_HPP
#define SOLRAC_DRIVERS_STM32_FDCAN_PHY_HPP

#include "can_phy.hpp"

// Forward-declare STM32 HAL FDCAN handle so this header compiles on any target
struct __FDCAN_HandleTypeDef;
using FDCAN_HandleTypeDef = struct __FDCAN_HandleTypeDef;

namespace solrac
{

    /// @brief Physical CAN-FD controller driver for STM32 internal FDCAN peripherals.
    ///
    /// @details
    /// RFC 2119 Requirements:
    /// - Integrators MUST configure FDCAN clock, bit timings, and 29-bit extended filters
    ///   in STM32CubeMX (or equivalent HAL init) prior to calling init().
    /// - Implementations MUST NOT block or allocate dynamic memory.
    class Stm32FdcanPhy final : public ICanPhy
    {
    public:
        explicit Stm32FdcanPhy(FDCAN_HandleTypeDef &hfdcan) noexcept;
        ~Stm32FdcanPhy() noexcept override = default;

        Stm32FdcanPhy(const Stm32FdcanPhy &) = delete;
        Stm32FdcanPhy &operator=(const Stm32FdcanPhy &) = delete;

        [[nodiscard]] bool init() noexcept override;
        [[nodiscard]] bool transmit(const CanFdFrame &frame) noexcept override;
        [[nodiscard]] bool receive(CanFdFrame &out_frame) noexcept override;
        [[nodiscard]] bool isHealthy() const noexcept override;

    private:
        FDCAN_HandleTypeDef &hfdcan_;
        bool started_{false};
    };

} // namespace solrac

#endif // SOLRAC_DRIVERS_STM32_FDCAN_PHY_HPP