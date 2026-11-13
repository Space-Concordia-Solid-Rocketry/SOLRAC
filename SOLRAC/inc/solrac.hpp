#ifndef SOLRAC_INC_SOLRAC_HPP
#define SOLRAC_INC_SOLRAC_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "can_phy.hpp"

namespace solrac
{

    /// @brief OpenCyphal Node-ID (valid range: 1..127; 255 = anonymous).
    /// @details Integrators MUST assign a unique NodeId to each node on a vehicle.
    using NodeId = std::uint8_t;

    /// @brief OpenCyphal Port-ID (Subject-ID: 0..8191, Service-ID: 0..511).
    /// @details Integrators define PortId constants in a shared vehicle network header.
    using PortId = std::uint16_t;

    /// @brief Hardware-arbitrated message priority (bits [28:26] of 29-bit CAN-FD ID).
    /// @details Lower numeric value = higher priority on the physical CAN bus.
    enum class Priority : std::uint8_t
    {
        kExceptional = 0U, ///< Abort, flight termination, critical safety interlocks
        kImmediate = 1U,   ///< Critical actuation and pyrotechnic commands
        kFast = 2U,        ///< High-rate guidance, navigation, and control commands
        kHigh = 3U,        ///< Real-time RF downlink telemetry frames
        kNominal = 4U,     ///< Standard sensor and power telemetry (default)
        kLow = 5U,         ///< Slow housekeeping (temperatures, status flags)
        kSlow = 6U,        ///< Background diagnostics
        kOptional = 7U     ///< Bulk log dumps and non-critical debug traffic
    };

    /// @brief Bus routing policy per Port-ID.
    /// @details
    /// RFC 2119 Routing Guarantees:
    /// - In 1-Bus Mode, all policies SHALL route over the single active bus.
    /// - In 2-Bus Mode:
    ///   - kDualCast:   MUST transmit on BOTH Bus 0 (TTC) and Bus 1 (GEN) simultaneously.
    ///   - kTtcPrimary: SHOULD transmit on TTC; MUST fail over to GEN if TTC is unhealthy.
    ///   - kGenPrimary: SHOULD transmit on GEN; MUST fail over to TTC if GEN is unhealthy.
    ///   - kTtcOnly:    MUST transmit ONLY on TTC; SHALL NOT fail over to GEN.
    ///   - kGenOnly:    MUST transmit ONLY on GEN; SHALL NOT fail over to TTC.
    enum class RoutePolicy : std::uint8_t
    {
        kDualCast = 0U,
        kBus0Primary = 1U,
        kBus1Primary = 2U,
        kBus0Only = 3U,
        kBus1Only = 4U,

        // Semantic aliases for TTC (Bus 0) and GEN (Bus 1) lines
        kTtcPrimary = kBus0Primary,
        kGenPrimary = kBus1Primary,
        kTtcOnly = kBus0Only,
        kGenOnly = kBus1Only
    };

    /// @brief Declarative routing and priority entry for one Port-ID.
    struct PortConfig final
    {
        PortId port_id{0U};
        Priority priority{Priority::kNominal};
        RoutePolicy route{RoutePolicy::kBus0Primary};
    };

    /// @brief Deterministic status codes (replaces C++ exceptions per JSF AV Rule 208).
    enum class Status : std::uint8_t
    {
        kOk = 0U,
        kNotInitialized = 1U,
        kInvalidArgument = 2U,
        kPayloadTooLarge = 3U,
        kRegistryFull = 4U,
        kQueueFull = 5U,
        kTargetBusUnavailable = 6U
    };

    /// @brief Metadata passed to application callbacks upon receiving a complete transfer.
    struct TransferInfo final
    {
        PortId port_id{0U};
        NodeId source_node{0U};
        std::uint8_t transfer_id{0U};
        Priority priority{Priority::kNominal};
        std::uint64_t timestamp_usec{0U};
    };

    /// @brief Callback signature for incoming Pub/Sub messages (zero heap allocation).
    using MessageCallback = void (*)(const TransferInfo &info,
                                     const std::uint8_t *payload,
                                     std::size_t payload_size,
                                     void *user_ctx) noexcept;

    /// @brief Callback signature for incoming RPC service requests or responses.
    using ServiceCallback = void (*)(const TransferInfo &info,
                                     const std::uint8_t *payload,
                                     std::size_t payload_size,
                                     void *user_ctx) noexcept;

    /// @brief Safely unpacks a received byte buffer into a trivially copyable C++ struct.
    template <typename T>
    [[nodiscard]] inline bool unpackPayload(const std::uint8_t *payload,
                                            std::size_t payload_size,
                                            T &out_msg) noexcept
    {
        static_assert(std::is_trivially_copyable_v<T>,
                      "SOLRAC messages MUST be trivially copyable (no pointers/std::vector)");
        static_assert(std::is_standard_layout_v<T>,
                      "SOLRAC messages MUST have standard memory layout");

        if ((payload == nullptr) || (payload_size != sizeof(T)))
        {
            return false;
        }
        std::memcpy(&out_msg, payload, sizeof(T));
        return true;
    }

    /// @brief Configurable 1-Bus or 2-Bus CAN-FD Avionics Transport Engine.
    ///
    /// @details
    /// RFC 2119 Operational Specification:
    /// 1. Applications MUST call init() once before registering routes, subscribing, or updating.
    /// 2. Zero dynamic heap allocations SHALL occur after init().
    /// 3. Integrators SHOULD call configurePorts() during startup to bind Port-IDs to priorities
    ///    and bus routing rules. Unconfigured ports SHALL default to (kNominal, kBus0Primary).
    class SolracBus final
    {
    public:
        static constexpr std::size_t kMaxPayloadBytes = 256U;
        static constexpr std::size_t kMaxSubscriptions = 32U;
        static constexpr std::size_t kMaxConfiguredPorts = 48U;
        static constexpr std::size_t kMaxTrackedTxPorts = 48U;
        static constexpr std::size_t kStaticArenaBytes = 16384U;

        /// @brief 1-Bus Mode Constructor.
        /// @details Operates in 1-Bus Mode; all traffic routes through single_phy.
        SolracBus(NodeId local_node, ICanPhy &single_phy) noexcept;

        /// @brief 2-Bus Mode Constructor (TTC and GEN buses, or TTC-only / GEN-only nodes).
        /// @param[in] local_node Unique NodeId of the host node.
        /// @param[in] bus0_phy   Pointer to Bus 0 (TTC) driver, or nullptr in GEN-only mode.
        /// @param[in] bus1_phy   Pointer to Bus 1 (GEN) driver, or nullptr in TTC-only mode.
        SolracBus(NodeId local_node, ICanPhy *bus0_phy, ICanPhy *bus1_phy) noexcept;

        ~SolracBus() noexcept = default;

        SolracBus(const SolracBus &) = delete;
        SolracBus &operator=(const SolracBus &) = delete;

        /// @brief Initializes the O(1) static heap and transport state.
        [[nodiscard]] Status init() noexcept;

        /// @brief Registers routing and priority rules for a single Port-ID.
        [[nodiscard]] Status configurePort(const PortConfig &config) noexcept;

        /// @brief Registers an entire array of PortConfig rules in one call at startup.
        [[nodiscard]] Status configurePorts(const PortConfig *configs, std::size_t count) noexcept;

        /// @brief Registers a handler to receive broadcast Pub/Sub messages on port_id.
        [[nodiscard]] Status subscribeMessage(PortId port_id,
                                              MessageCallback callback,
                                              void *user_ctx = nullptr,
                                              std::size_t max_payload_bytes = kMaxPayloadBytes) noexcept;

        /// @brief Registers a handler to receive unicast RPC requests on service_id.
        [[nodiscard]] Status subscribeServiceRequest(PortId service_id,
                                                     ServiceCallback callback,
                                                     void *user_ctx = nullptr,
                                                     std::size_t max_payload_bytes = kMaxPayloadBytes) noexcept;

        /// @brief Registers a handler to receive unicast RPC responses on service_id.
        [[nodiscard]] Status subscribeServiceResponse(PortId service_id,
                                                      ServiceCallback callback,
                                                      void *user_ctx = nullptr,
                                                      std::size_t max_payload_bytes = kMaxPayloadBytes) noexcept;

        /// @brief Publishes a struct using the Priority and RoutePolicy configured for subject_id.
        template <typename T>
        [[nodiscard]] Status publish(PortId subject_id,
                                     const T &msg,
                                     std::uint64_t now_usec,
                                     std::uint64_t timeout_usec = 50000ULL) noexcept
        {
            static_assert(std::is_trivially_copyable_v<T>, "Message struct MUST be trivially copyable");
            static_assert(std::is_standard_layout_v<T>, "Message struct MUST have standard layout");
            static_assert(sizeof(T) <= kMaxPayloadBytes, "Message struct exceeds kMaxPayloadBytes");

            const PortConfig cfg = lookupPortConfig(subject_id);
            return publishRaw(subject_id, cfg.priority, cfg.route, &msg, sizeof(T), now_usec, timeout_usec);
        }

        /// @brief Publishes a struct with an explicit Priority and RoutePolicy override.
        template <typename T>
        [[nodiscard]] Status publish(PortId subject_id,
                                     Priority priority,
                                     RoutePolicy route,
                                     const T &msg,
                                     std::uint64_t now_usec,
                                     std::uint64_t timeout_usec = 50000ULL) noexcept
        {
            static_assert(std::is_trivially_copyable_v<T>, "Message struct MUST be trivially copyable");
            static_assert(std::is_standard_layout_v<T>, "Message struct MUST have standard layout");
            static_assert(sizeof(T) <= kMaxPayloadBytes, "Message struct exceeds kMaxPayloadBytes");

            return publishRaw(subject_id, priority, route, &msg, sizeof(T), now_usec, timeout_usec);
        }

        /// @brief Sends a unicast RPC command using the Priority and RoutePolicy configured for service_id.
        template <typename T>
        [[nodiscard]] Status sendRequest(NodeId target_node,
                                         PortId service_id,
                                         const T &req,
                                         std::uint64_t now_usec,
                                         std::uint64_t timeout_usec = 50000ULL) noexcept
        {
            static_assert(std::is_trivially_copyable_v<T>, "Request struct MUST be trivially copyable");
            static_assert(std::is_standard_layout_v<T>, "Request struct MUST have standard layout");
            static_assert(sizeof(T) <= kMaxPayloadBytes, "Request struct exceeds kMaxPayloadBytes");

            const PortConfig cfg = lookupPortConfig(service_id);
            return sendRequestRaw(target_node, service_id, cfg.priority, cfg.route, &req, sizeof(T), now_usec, timeout_usec);
        }

        /// @brief Sends a unicast RPC command with an explicit Priority and RoutePolicy override.
        template <typename T>
        [[nodiscard]] Status sendRequest(NodeId target_node,
                                         PortId service_id,
                                         Priority priority,
                                         RoutePolicy route,
                                         const T &req,
                                         std::uint64_t now_usec,
                                         std::uint64_t timeout_usec = 50000ULL) noexcept
        {
            static_assert(std::is_trivially_copyable_v<T>, "Request struct MUST be trivially copyable");
            static_assert(std::is_standard_layout_v<T>, "Request struct MUST have standard layout");
            static_assert(sizeof(T) <= kMaxPayloadBytes, "Request struct exceeds kMaxPayloadBytes");

            return sendRequestRaw(target_node, service_id, priority, route, &req, sizeof(T), now_usec, timeout_usec);
        }

        /// @brief Sends a unicast RPC response using the Priority and RoutePolicy configured for service_id.
        template <typename T>
        [[nodiscard]] Status sendResponse(NodeId requester_node,
                                          PortId service_id,
                                          std::uint8_t request_transfer_id,
                                          const T &resp,
                                          std::uint64_t now_usec,
                                          std::uint64_t timeout_usec = 50000ULL) noexcept
        {
            static_assert(std::is_trivially_copyable_v<T>, "Response struct MUST be trivially copyable");
            static_assert(std::is_standard_layout_v<T>, "Response struct MUST have standard layout");
            static_assert(sizeof(T) <= kMaxPayloadBytes, "Response struct exceeds kMaxPayloadBytes");

            const PortConfig cfg = lookupPortConfig(service_id);
            return sendResponseRaw(requester_node, service_id, request_transfer_id, cfg.priority, cfg.route, &resp, sizeof(T), now_usec, timeout_usec);
        }

        /// @brief Raw byte-buffer transmit methods (used by templates or custom serializers).
        [[nodiscard]] Status publishRaw(PortId subject_id,
                                        Priority priority,
                                        RoutePolicy route,
                                        const void *payload,
                                        std::size_t payload_size,
                                        std::uint64_t now_usec,
                                        std::uint64_t timeout_usec = 50000ULL) noexcept;

        [[nodiscard]] Status sendRequestRaw(NodeId target_node,
                                            PortId service_id,
                                            Priority priority,
                                            RoutePolicy route,
                                            const void *payload,
                                            std::size_t payload_size,
                                            std::uint64_t now_usec,
                                            std::uint64_t timeout_usec = 50000ULL) noexcept;

        [[nodiscard]] Status sendResponseRaw(NodeId requester_node,
                                             PortId service_id,
                                             std::uint8_t request_transfer_id,
                                             Priority priority,
                                             RoutePolicy route,
                                             const void *payload,
                                             std::size_t payload_size,
                                             std::uint64_t now_usec,
                                             std::uint64_t timeout_usec = 50000ULL) noexcept;

        /// @brief Services all connected CAN-FD buses (RX polling, deduplication, callbacks, TX flushing).
        void update(std::uint64_t now_usec) noexcept;

        /// @brief Returns true if operating in 1-Bus Mode.
        [[nodiscard]] bool isSingleBusMode() const noexcept { return single_bus_mode_; }

        /// @brief Diagnostic counters for flight logging and ground testing.
        [[nodiscard]] std::uint32_t getTxDropCount() const noexcept { return tx_drop_count_; }
        [[nodiscard]] std::uint32_t getRxCompleteCount() const noexcept { return rx_complete_count_; }

    private:
        struct PortRouteEntry final
        {
            bool in_use{false};
            PortConfig config{};
        };

        struct TxTransferIdEntry final
        {
            bool in_use{false};
            std::uint8_t kind{0U};
            PortId port_id{0U};
            NodeId remote_node{0U};
            std::uint8_t next_transfer_id{0U};
        };

        [[nodiscard]] PortConfig lookupPortConfig(PortId port_id) const noexcept;

        std::uint8_t allocateTransferId(std::uint8_t kind, PortId port_id, NodeId remote_node) noexcept;

        Status enqueueTransfer(std::uint8_t transfer_kind,
                               PortId port_id,
                               NodeId remote_node,
                               std::uint8_t transfer_id,
                               Priority priority,
                               RoutePolicy route,
                               const void *payload,
                               std::size_t payload_size,
                               std::uint64_t now_usec,
                               std::uint64_t timeout_usec) noexcept;

        NodeId local_node_;
        ICanPhy *phys_[2]{nullptr, nullptr};
        bool single_bus_mode_{false};
        bool initialized_{false};
        std::uint32_t tx_drop_count_{0U};
        std::uint32_t rx_complete_count_{0U};

        PortRouteEntry route_table_[kMaxConfiguredPorts]{};
        TxTransferIdEntry tx_id_table_[kMaxTrackedTxPorts]{};

        alignas(16) std::uint8_t arena_[kStaticArenaBytes]{};
        alignas(16) std::uint8_t runtime_storage_[6144]{};
    };

} // namespace solrac

#endif // SOLRAC_INC_SOLRAC_HPP