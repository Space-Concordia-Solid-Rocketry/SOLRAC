#ifndef SOLRAC_DRIVERS_CAN_PHY_HPP
#define SOLRAC_DRIVERS_CAN_PHY_HPP

#include <cstddef>
#include <cstdint>

namespace solrac
{

    /// @brief Maximum payload size of a single CAN-FD frame in bytes.
    inline constexpr std::size_t kCanFdMaxPayloadBytes = 64U;

    /// @brief Physical bus index in the dual-redundant avionics architecture.
    enum class BusId : std::uint8_t
    {
        kTtc = 0U, ///< Telemetry, Tracking & Command bus
        kGen = 1U  ///< General telemetry & redundant backup bus
    };

    /// @brief Hardware-independent CAN-FD frame container.
    struct CanFdFrame final
    {
        std::uint32_t extended_id{0U};                 ///< 29-bit CAN-FD identifier
        std::uint8_t payload_size{0U};                 ///< Valid payload length (0..64)
        std::uint8_t payload[kCanFdMaxPayloadBytes]{}; ///< Fixed-size frame buffer
    };

    /// @brief Abstract physical CAN-FD controller interface (JSF AV C++ compliant).
    ///
    /// @details
    /// RFC 2119 Requirements:
    /// - Implementations MUST NOT throw exceptions or allocate heap memory.
    /// - Implementations MUST be non-blocking in transmit() and receive().
    /// - Implementations SHALL return false from isHealthy() when in BUS_OFF,
    ///   error-passive, or controller SPI fault states.
    class ICanPhy
    {
    public:
        virtual ~ICanPhy() noexcept = default;

        /// @brief Initializes the CAN-FD controller hardware.
        [[nodiscard]] virtual bool init() noexcept = 0;

        /// @brief Places one CAN-FD frame into a hardware TX mailbox/FIFO.
        /// @return true if queued into hardware; false if TX FIFO is full or faulted.
        [[nodiscard]] virtual bool transmit(const CanFdFrame &frame) noexcept = 0;

        /// @brief Reads one received CAN-FD frame from the hardware RX FIFO.
        /// @return true if a frame was retrieved; false if RX FIFO is empty.
        [[nodiscard]] virtual bool receive(CanFdFrame &out_frame) noexcept = 0;

        /// @brief Reports whether this physical bus is currently operational.
        [[nodiscard]] virtual bool isHealthy() const noexcept = 0;

    protected:
        ICanPhy() noexcept = default;
        ICanPhy(const ICanPhy &) = delete;
        ICanPhy &operator=(const ICanPhy &) = delete;
    };

} // namespace solrac

#endif // SOLRAC_DRIVERS_CAN_PHY_HPP