#include "solrac.hpp"

#include <cstring>
#include <new>

#include "canard.h"
#include "o1heap.h"

namespace solrac
{
    namespace
    {

        constexpr std::size_t kTxQueueCapacityFrames = 64U;
        constexpr canard_us_t kDefaultRxTimeoutUsec = CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_us;
        constexpr std::uint8_t kIfaceBus0Mask = 0x01U; // TT&C (or single bus)
        constexpr std::uint8_t kIfaceBus1Mask = 0x02U; // GEN

        // ---------------------------------------------------------------------------
        // O1Heap <-> Libcanard v5.0 memory vtable bridge (JSF AV C++ compliant)
        // ---------------------------------------------------------------------------
        void *o1heapCanardAlloc(const canard_mem_t mem, const std::size_t size) noexcept
        {
            if (mem.context == nullptr)
            {
                return nullptr;
            }
            return o1heapAllocate(static_cast<O1HeapInstance *>(mem.context), size);
        }

        void o1heapCanardFree(const canard_mem_t mem, const std::size_t /*size*/, void *const ptr) noexcept
        {
            if ((mem.context != nullptr) && (ptr != nullptr))
            {
                o1heapFree(static_cast<O1HeapInstance *>(mem.context), ptr);
            }
        }

        inline constexpr canard_mem_vtable_t kO1HeapMemVtable{
            &o1heapCanardFree,
            &o1heapCanardAlloc};

        struct SubscriptionSlot final
        {
            bool in_use{false};
            canard_kind_t kind{canard_kind_message_16b};
            PortId port_id{0U};
            MessageCallback msg_cb{nullptr};
            ServiceCallback svc_cb{nullptr};
            void *user_ctx{nullptr};
            std::uint32_t *rx_count_ptr{nullptr};
            canard_subscription_t canard_sub{};
        };

        struct RuntimeState final
        {
            O1HeapInstance *heap{nullptr};
            canard_t canard{};
            ICanPhy *phys[2]{nullptr, nullptr};
            std::uint8_t available_iface_bitmap{0U};
            canard_us_t current_time_usec{0};
            std::uint32_t *tx_drop_count{nullptr};
            std::uint32_t *rx_complete_count{nullptr};
            SubscriptionSlot subs[SolracBus::kMaxSubscriptions]{};
        };

        // ---------------------------------------------------------------------------
        // Libcanard v5.0 transport callbacks
        // ---------------------------------------------------------------------------
        canard_us_t canardNowCallback(const canard_t *const self) noexcept
        {
            if ((self == nullptr) || (self->user_context == nullptr))
            {
                return 0;
            }
            const auto *const rt = static_cast<const RuntimeState *>(self->user_context);
            return rt->current_time_usec;
        }

        bool canardTxCallback(canard_t *const self,
                              void *const /*user_context*/,
                              const canard_us_t deadline,
                              const std::uint_least8_t iface_index,
                              const bool /*fd*/,
                              const std::uint32_t extended_can_id,
                              const canard_bytes_t can_data) noexcept
        {
            if ((self == nullptr) || (self->user_context == nullptr) || (iface_index >= 2U))
            {
                return false;
            }

            auto *const rt = static_cast<RuntimeState *>(self->user_context);
            if (rt->phys[iface_index] == nullptr)
            {
                return false;
            }

            // Drop expired frames so stale commands never execute late
            if (deadline < rt->current_time_usec)
            {
                if (rt->tx_drop_count != nullptr)
                {
                    (*rt->tx_drop_count)++;
                }
                return true;
            }

            if (can_data.size > kCanFdMaxPayloadBytes)
            {
                return false;
            }

            CanFdFrame frame{};
            frame.extended_id = extended_can_id;
            frame.payload_size = static_cast<std::uint8_t>(can_data.size);
            if ((can_data.data != nullptr) && (frame.payload_size > 0U))
            {
                std::memcpy(frame.payload, can_data.data, frame.payload_size);
            }

            return rt->phys[iface_index]->transmit(frame);
        }

        inline constexpr canard_vtable_t kCanardVtable{
            &canardNowCallback,
            &canardTxCallback,
            nullptr // Software subscription filtering used; hardware filter callback optional
        };

        void onSubscriptionMessage(canard_subscription_t *const self,
                                   const canard_us_t timestamp,
                                   const canard_prio_t priority,
                                   const std::uint_least8_t source_node_id,
                                   const std::uint_least8_t transfer_id,
                                   const canard_payload_t payload) noexcept
        {
            if ((self == nullptr) || (self->user_context == nullptr))
            {
                return;
            }

            auto *const slot = static_cast<SubscriptionSlot *>(self->user_context);
            if (slot->rx_count_ptr != nullptr)
            {
                (*slot->rx_count_ptr)++;
            }

            TransferInfo info{};
            info.port_id = slot->port_id;
            info.source_node = source_node_id;
            info.transfer_id = transfer_id;
            info.priority = static_cast<Priority>(priority);
            info.timestamp_usec = (timestamp >= 0) ? static_cast<std::uint64_t>(timestamp) : 0ULL;

            const auto *const data_ptr = static_cast<const std::uint8_t *>(payload.view.data);
            const std::size_t data_len = payload.view.size;

            if ((slot->kind == canard_kind_message_16b) && (slot->msg_cb != nullptr))
            {
                slot->msg_cb(info, data_ptr, data_len, slot->user_ctx);
            }
            else if (slot->svc_cb != nullptr)
            {
                slot->svc_cb(info, data_ptr, data_len, slot->user_ctx);
            }

            // Release multi-frame reassembly buffer back to o1heap if allocated
            if ((payload.origin.data != nullptr) && (payload.origin.size > 0U) && (self->owner != nullptr))
            {
                self->owner->mem.rx_payload.vtable->free(
                    self->owner->mem.rx_payload, payload.origin.size, payload.origin.data);
            }
        }

        inline constexpr canard_subscription_vtable_t kSubscriptionVtable{
            &onSubscriptionMessage};

    } // namespace

    // 1-Bus Mode Constructor
    SolracBus::SolracBus(const NodeId local_node, ICanPhy &single_phy) noexcept
        : local_node_(local_node), single_bus_mode_(true)
    {
        phys_[0] = &single_phy;
        phys_[1] = nullptr;

        static_assert(sizeof(RuntimeState) <= sizeof(runtime_storage_),
                      "Increase SolracBus::runtime_storage_ size to hold RuntimeState");
        static_assert((kStaticArenaBytes % O1HEAP_ALIGNMENT) == 0U,
                      "kStaticArenaBytes MUST be a multiple of O1HEAP_ALIGNMENT");
    }

    // 2-Bus Mode Constructor (TTC and GEN buses, or TTC-only / GEN-only nodes)
    SolracBus::SolracBus(const NodeId local_node, ICanPhy *const bus0_phy, ICanPhy *const bus1_phy) noexcept
        : local_node_(local_node), single_bus_mode_(false)
    {
        phys_[0] = bus0_phy;
        phys_[1] = bus1_phy;
    }

    Status SolracBus::init() noexcept
    {
        if ((local_node_ == 0U) || (local_node_ > CANARD_NODE_ID_MAX))
        {
            return Status::kInvalidArgument;
        }
        if ((phys_[0] == nullptr) && (phys_[1] == nullptr))
        {
            return Status::kInvalidArgument;
        }

        auto *const rt = new (runtime_storage_) RuntimeState();
        rt->phys[0] = phys_[0];
        rt->phys[1] = phys_[1];
        rt->tx_drop_count = &tx_drop_count_;
        rt->rx_complete_count = &rx_complete_count_;

        rt->heap = o1heapInit(arena_, sizeof(arena_));
        if (rt->heap == nullptr)
        {
            return Status::kNotInitialized;
        }

        const canard_mem_t mem_res{&kO1HeapMemVtable, rt->heap};
        const canard_mem_set_t mem_set{mem_res, mem_res, mem_res, mem_res, mem_res};

        std::uint8_t iface_bitmap = 0U;
        if (phys_[0] != nullptr)
        {
            iface_bitmap = static_cast<std::uint8_t>(iface_bitmap | kIfaceBus0Mask);
        }
        if (phys_[1] != nullptr)
        {
            iface_bitmap = static_cast<std::uint8_t>(iface_bitmap | kIfaceBus1Mask);
        }
        rt->available_iface_bitmap = iface_bitmap;

        const std::uint64_t prng_seed =
            static_cast<std::uint64_t>(local_node_) * 0x9E3779B97F4A7C15ULL;

        if (!canard_new(&rt->canard,
                        &kCanardVtable,
                        mem_set,
                        iface_bitmap,
                        kTxQueueCapacityFrames,
                        prng_seed,
                        0U))
        {
            return Status::kNotInitialized;
        }

        rt->canard.user_context = rt;

        if (!canard_set_node_id(&rt->canard, local_node_))
        {
            return Status::kInvalidArgument;
        }

        initialized_ = true;
        return Status::kOk;
    }

    Status SolracBus::configurePort(const PortConfig &config) noexcept
    {
        for (std::size_t i = 0U; i < kMaxConfiguredPorts; ++i)
        {
            if (route_table_[i].in_use && (route_table_[i].config.port_id == config.port_id))
            {
                route_table_[i].config = config;
                return Status::kOk;
            }
        }
        for (std::size_t i = 0U; i < kMaxConfiguredPorts; ++i)
        {
            if (!route_table_[i].in_use)
            {
                route_table_[i].in_use = true;
                route_table_[i].config = config;
                return Status::kOk;
            }
        }
        return Status::kRegistryFull;
    }

    Status SolracBus::configurePorts(const PortConfig *const configs, const std::size_t count) noexcept
    {
        if ((configs == nullptr) && (count > 0U))
        {
            return Status::kInvalidArgument;
        }
        for (std::size_t i = 0U; i < count; ++i)
        {
            const Status st = configurePort(configs[i]);
            if (st != Status::kOk)
            {
                return st;
            }
        }
        return Status::kOk;
    }

    PortConfig SolracBus::lookupPortConfig(const PortId port_id) const noexcept
    {
        for (std::size_t i = 0U; i < kMaxConfiguredPorts; ++i)
        {
            if (route_table_[i].in_use && (route_table_[i].config.port_id == port_id))
            {
                return route_table_[i].config;
            }
        }
        return PortConfig{port_id, Priority::kNominal, RoutePolicy::kBus0Primary};
    }

    Status SolracBus::subscribeMessage(const PortId port_id,
                                       const MessageCallback callback,
                                       void *const user_ctx,
                                       const std::size_t max_payload_bytes) noexcept
    {
        if (!initialized_)
        {
            return Status::kNotInitialized;
        }
        if ((callback == nullptr) || (max_payload_bytes > kMaxPayloadBytes))
        {
            return Status::kInvalidArgument;
        }

        auto *const rt = reinterpret_cast<RuntimeState *>(runtime_storage_);
        for (std::size_t i = 0U; i < kMaxSubscriptions; ++i)
        {
            if (!rt->subs[i].in_use)
            {
                rt->subs[i].in_use = true;
                rt->subs[i].kind = canard_kind_message_16b;
                rt->subs[i].port_id = port_id;
                rt->subs[i].msg_cb = callback;
                rt->subs[i].svc_cb = nullptr;
                rt->subs[i].user_ctx = user_ctx;
                rt->subs[i].rx_count_ptr = &rx_complete_count_;

                canard_subscription_t *const sub = canard_subscribe_16b(&rt->canard,
                                                                        &rt->subs[i].canard_sub,
                                                                        port_id,
                                                                        max_payload_bytes,
                                                                        kDefaultRxTimeoutUsec,
                                                                        &kSubscriptionVtable);
                if (sub == nullptr)
                {
                    rt->subs[i].in_use = false;
                    return Status::kInvalidArgument;
                }
                sub->user_context = &rt->subs[i];
                return Status::kOk;
            }
        }
        return Status::kRegistryFull;
    }

    Status SolracBus::subscribeServiceRequest(const PortId service_id,
                                              const ServiceCallback callback,
                                              void *const user_ctx,
                                              const std::size_t max_payload_bytes) noexcept
    {
        if (!initialized_)
        {
            return Status::kNotInitialized;
        }
        if ((callback == nullptr) || (max_payload_bytes > kMaxPayloadBytes))
        {
            return Status::kInvalidArgument;
        }

        auto *const rt = reinterpret_cast<RuntimeState *>(runtime_storage_);
        for (std::size_t i = 0U; i < kMaxSubscriptions; ++i)
        {
            if (!rt->subs[i].in_use)
            {
                rt->subs[i].in_use = true;
                rt->subs[i].kind = canard_kind_request;
                rt->subs[i].port_id = service_id;
                rt->subs[i].msg_cb = nullptr;
                rt->subs[i].svc_cb = callback;
                rt->subs[i].user_ctx = user_ctx;
                rt->subs[i].rx_count_ptr = &rx_complete_count_;

                canard_subscription_t *const sub = canard_subscribe_request(&rt->canard,
                                                                            &rt->subs[i].canard_sub,
                                                                            service_id,
                                                                            max_payload_bytes,
                                                                            kDefaultRxTimeoutUsec,
                                                                            &kSubscriptionVtable);
                if (sub == nullptr)
                {
                    rt->subs[i].in_use = false;
                    return Status::kInvalidArgument;
                }
                sub->user_context = &rt->subs[i];
                return Status::kOk;
            }
        }
        return Status::kRegistryFull;
    }

    Status SolracBus::subscribeServiceResponse(const PortId service_id,
                                               const ServiceCallback callback,
                                               void *const user_ctx,
                                               const std::size_t max_payload_bytes) noexcept
    {
        if (!initialized_)
        {
            return Status::kNotInitialized;
        }
        if ((callback == nullptr) || (max_payload_bytes > kMaxPayloadBytes))
        {
            return Status::kInvalidArgument;
        }

        auto *const rt = reinterpret_cast<RuntimeState *>(runtime_storage_);
        for (std::size_t i = 0U; i < kMaxSubscriptions; ++i)
        {
            if (!rt->subs[i].in_use)
            {
                rt->subs[i].in_use = true;
                rt->subs[i].kind = canard_kind_response;
                rt->subs[i].port_id = service_id;
                rt->subs[i].msg_cb = nullptr;
                rt->subs[i].svc_cb = callback;
                rt->subs[i].user_ctx = user_ctx;
                rt->subs[i].rx_count_ptr = &rx_complete_count_;

                canard_subscription_t *const sub = canard_subscribe_response(&rt->canard,
                                                                             &rt->subs[i].canard_sub,
                                                                             service_id,
                                                                             max_payload_bytes,
                                                                             &kSubscriptionVtable);
                if (sub == nullptr)
                {
                    rt->subs[i].in_use = false;
                    return Status::kInvalidArgument;
                }
                sub->user_context = &rt->subs[i];
                return Status::kOk;
            }
        }
        return Status::kRegistryFull;
    }

    std::uint8_t SolracBus::allocateTransferId(const std::uint8_t kind,
                                               const PortId port_id,
                                               const NodeId remote_node) noexcept
    {
        for (std::size_t i = 0U; i < kMaxTrackedTxPorts; ++i)
        {
            if (tx_id_table_[i].in_use &&
                (tx_id_table_[i].kind == kind) &&
                (tx_id_table_[i].port_id == port_id) &&
                (tx_id_table_[i].remote_node == remote_node))
            {
                const std::uint8_t id = tx_id_table_[i].next_transfer_id;
                tx_id_table_[i].next_transfer_id =
                    static_cast<std::uint8_t>((id + 1U) & CANARD_TRANSFER_ID_MAX);
                return id;
            }
        }

        for (std::size_t i = 0U; i < kMaxTrackedTxPorts; ++i)
        {
            if (!tx_id_table_[i].in_use)
            {
                tx_id_table_[i].in_use = true;
                tx_id_table_[i].kind = kind;
                tx_id_table_[i].port_id = port_id;
                tx_id_table_[i].remote_node = remote_node;
                tx_id_table_[i].next_transfer_id = 1U;
                return 0U;
            }
        }

        return 0U;
    }

    Status SolracBus::publishRaw(const PortId subject_id,
                                 const Priority priority,
                                 const RoutePolicy route,
                                 const void *const payload,
                                 const std::size_t payload_size,
                                 const std::uint64_t now_usec,
                                 const std::uint64_t timeout_usec) noexcept
    {
        const std::uint8_t tid = allocateTransferId(
            static_cast<std::uint8_t>(canard_kind_message_16b), subject_id, CANARD_NODE_ID_ANONYMOUS);
        return enqueueTransfer(static_cast<std::uint8_t>(canard_kind_message_16b),
                               subject_id,
                               CANARD_NODE_ID_ANONYMOUS,
                               tid,
                               priority,
                               route,
                               payload,
                               payload_size,
                               now_usec,
                               timeout_usec);
    }

    Status SolracBus::sendRequestRaw(const NodeId target_node,
                                     const PortId service_id,
                                     const Priority priority,
                                     const RoutePolicy route,
                                     const void *const payload,
                                     const std::size_t payload_size,
                                     const std::uint64_t now_usec,
                                     const std::uint64_t timeout_usec) noexcept
    {
        if ((target_node == 0U) || (target_node > CANARD_NODE_ID_MAX))
        {
            return Status::kInvalidArgument;
        }
        const std::uint8_t tid = allocateTransferId(
            static_cast<std::uint8_t>(canard_kind_request), service_id, target_node);
        return enqueueTransfer(static_cast<std::uint8_t>(canard_kind_request),
                               service_id,
                               target_node,
                               tid,
                               priority,
                               route,
                               payload,
                               payload_size,
                               now_usec,
                               timeout_usec);
    }

    Status SolracBus::sendResponseRaw(const NodeId requester_node,
                                      const PortId service_id,
                                      const std::uint8_t request_transfer_id,
                                      const Priority priority,
                                      const RoutePolicy route,
                                      const void *const payload,
                                      const std::size_t payload_size,
                                      const std::uint64_t now_usec,
                                      const std::uint64_t timeout_usec) noexcept
    {
        if ((requester_node == 0U) || (requester_node > CANARD_NODE_ID_MAX))
        {
            return Status::kInvalidArgument;
        }
        return enqueueTransfer(static_cast<std::uint8_t>(canard_kind_response),
                               service_id,
                               requester_node,
                               static_cast<std::uint8_t>(request_transfer_id & CANARD_TRANSFER_ID_MAX),
                               priority,
                               route,
                               payload,
                               payload_size,
                               now_usec,
                               timeout_usec);
    }

    Status SolracBus::enqueueTransfer(const std::uint8_t transfer_kind,
                                      const PortId port_id,
                                      const NodeId remote_node,
                                      const std::uint8_t transfer_id,
                                      const Priority priority,
                                      const RoutePolicy route,
                                      const void *const payload,
                                      const std::size_t payload_size,
                                      const std::uint64_t now_usec,
                                      const std::uint64_t timeout_usec) noexcept
    {
        if (!initialized_)
        {
            return Status::kNotInitialized;
        }
        if ((payload == nullptr) && (payload_size > 0U))
        {
            return Status::kInvalidArgument;
        }
        if (payload_size > kMaxPayloadBytes)
        {
            return Status::kPayloadTooLarge;
        }

        const bool bus0_ok = (phys_[0] != nullptr) && phys_[0]->isHealthy();
        const bool bus1_ok = (phys_[1] != nullptr) && phys_[1]->isHealthy();

        std::uint8_t target_bitmap = 0U;

        if (single_bus_mode_)
        {
            // 1-Bus Mode: route all traffic over Bus 0
            if (bus0_ok)
            {
                target_bitmap = kIfaceBus0Mask;
            }
        }
        else
        {
            // 2-Bus Mode: route according to RoutePolicy across TTC (Bus 0) and GEN (Bus 1)
            switch (route)
            {
            case RoutePolicy::kDualCast:
                if (bus0_ok)
                {
                    target_bitmap = static_cast<std::uint8_t>(target_bitmap | kIfaceBus0Mask);
                }
                if (bus1_ok)
                {
                    target_bitmap = static_cast<std::uint8_t>(target_bitmap | kIfaceBus1Mask);
                }
                break;
            case RoutePolicy::kBus0Primary:
                if (bus0_ok)
                {
                    target_bitmap = kIfaceBus0Mask;
                }
                else if (bus1_ok)
                {
                    target_bitmap = kIfaceBus1Mask;
                }
                break;
            case RoutePolicy::kBus1Primary:
                if (bus1_ok)
                {
                    target_bitmap = kIfaceBus1Mask;
                }
                else if (bus0_ok)
                {
                    target_bitmap = kIfaceBus0Mask;
                }
                break;
            case RoutePolicy::kBus0Only:
                if (bus0_ok)
                {
                    target_bitmap = kIfaceBus0Mask;
                }
                break;
            case RoutePolicy::kBus1Only:
                if (bus1_ok)
                {
                    target_bitmap = kIfaceBus1Mask;
                }
                break;
            default:
                return Status::kInvalidArgument;
            }
        }

        if (target_bitmap == 0U)
        {
            return Status::kTargetBusUnavailable;
        }

        auto *const rt = reinterpret_cast<RuntimeState *>(runtime_storage_);
        rt->current_time_usec = static_cast<canard_us_t>(now_usec);

        const canard_us_t deadline = static_cast<canard_us_t>(now_usec + timeout_usec);
        const canard_prio_t prio = static_cast<canard_prio_t>(priority);

        canard_bytes_chain_t chain{};
        chain.bytes.size = payload_size;
        chain.bytes.data = payload;
        chain.next = nullptr;

        bool enqueued = false;
        const auto kind = static_cast<canard_kind_t>(transfer_kind);

        if (kind == canard_kind_message_16b)
        {
            enqueued = canard_publish_16b(&rt->canard,
                                          deadline,
                                          target_bitmap,
                                          prio,
                                          port_id,
                                          transfer_id,
                                          chain,
                                          nullptr);
        }
        else if (kind == canard_kind_request)
        {
            // Mask node interfaces temporarily so service routing obeys RoutePolicy
            rt->canard.iface_bitmap = target_bitmap;
            enqueued = canard_request(&rt->canard,
                                      deadline,
                                      prio,
                                      port_id,
                                      remote_node,
                                      transfer_id,
                                      chain,
                                      nullptr);
            rt->canard.iface_bitmap = rt->available_iface_bitmap;
        }
        else if (kind == canard_kind_response)
        {
            rt->canard.iface_bitmap = target_bitmap;
            enqueued = canard_respond(&rt->canard,
                                      deadline,
                                      prio,
                                      port_id,
                                      remote_node,
                                      transfer_id,
                                      chain,
                                      nullptr);
            rt->canard.iface_bitmap = rt->available_iface_bitmap;
        }
        else
        {
            return Status::kInvalidArgument;
        }

        if (!enqueued)
        {
            ++tx_drop_count_;
            return Status::kQueueFull;
        }
        return Status::kOk;
    }

    void SolracBus::update(const std::uint64_t now_usec) noexcept
    {
        if (!initialized_)
        {
            return;
        }

        auto *const rt = reinterpret_cast<RuntimeState *>(runtime_storage_);
        rt->current_time_usec = static_cast<canard_us_t>(now_usec);

        // 1. Ingest incoming frames from all connected physical buses (Bus 0 and/or Bus 1)
        for (std::uint8_t iface = 0U; iface < 2U; ++iface)
        {
            if (phys_[iface] == nullptr)
            {
                continue;
            }

            CanFdFrame raw_rx{};
            while (phys_[iface]->receive(raw_rx))
            {
                canard_bytes_t rx_bytes{};
                rx_bytes.size = raw_rx.payload_size;
                rx_bytes.data = raw_rx.payload;

                (void)canard_ingest_frame(&rt->canard,
                                          rt->current_time_usec,
                                          iface,
                                          raw_rx.extended_id,
                                          rx_bytes);
            }
        }

        // 2. Poll priority-ordered TX queues for all healthy connected interfaces
        std::uint8_t ready_bitmap = 0U;
        if ((phys_[0] != nullptr) && phys_[0]->isHealthy())
        {
            ready_bitmap = static_cast<std::uint8_t>(ready_bitmap | kIfaceBus0Mask);
        }
        if ((phys_[1] != nullptr) && phys_[1]->isHealthy())
        {
            ready_bitmap = static_cast<std::uint8_t>(ready_bitmap | kIfaceBus1Mask);
        }

        canard_poll(&rt->canard, ready_bitmap);
    }

} // namespace solrac