#include <cassert>
#include <cstdio>
#include <cstring>

#include "solrac.hpp"

// ============================================================================
// EXAMPLE VEHICLE NETWORK CONFIG (Defined once by the integrating team)
// ============================================================================
namespace vehicle_network_example
{

    inline constexpr solrac::NodeId kPrimaryControllerNode = 10U;
    inline constexpr solrac::NodeId kActuatorNode = 20U;
    inline constexpr solrac::NodeId kTtcOnlyNode = 30U;
    inline constexpr solrac::NodeId kGenOnlyNode = 40U;

    inline constexpr solrac::PortId kStateTelemPort = 100U;
    inline constexpr solrac::PortId kGenTelemetryPort = 101U;
    inline constexpr solrac::PortId kGenOnlyStatusPort = 102U;
    inline constexpr solrac::PortId kTtcOnlyStatusPort = 103U;
    inline constexpr solrac::PortId kCriticalCommandPort = 200U;

    // Declarative routing table binding Port-IDs to hardware priority and bus policy
    inline constexpr solrac::PortConfig kRoutingTable[] = {
        {kCriticalCommandPort, solrac::Priority::kImmediate, solrac::RoutePolicy::kDualCast},
        {kStateTelemPort, solrac::Priority::kHigh, solrac::RoutePolicy::kTtcPrimary},
        {kGenTelemetryPort, solrac::Priority::kNominal, solrac::RoutePolicy::kGenPrimary},
        {kGenOnlyStatusPort, solrac::Priority::kLow, solrac::RoutePolicy::kGenOnly},
        {kTtcOnlyStatusPort, solrac::Priority::kHigh, solrac::RoutePolicy::kTtcOnly}};

    struct __attribute__((packed)) StateTelemetryMsg
    {
        std::uint8_t state;
        float altitude_m;
    };

    struct __attribute__((packed)) NodeStatusMsg
    {
        std::uint8_t status_code;
        std::uint16_t metric;
    };

    struct __attribute__((packed)) CriticalCommandReq
    {
        std::uint8_t channel;
        std::uint8_t arm_key;
    };

} // namespace vehicle_network_example

// ============================================================================
// VIRTUAL CAN-FD HARNESS FOR PC SIMULATION
// ============================================================================
class VirtualCanWire final
{
public:
    static constexpr std::size_t kCapacity = 32U;
    solrac::CanFdFrame frames[kCapacity]{};
    std::size_t head{0U};
    std::size_t tail{0U};

    bool push(const solrac::CanFdFrame &f) noexcept
    {
        const std::size_t next = (head + 1U) % kCapacity;
        if (next == tail)
        {
            return false;
        }
        frames[head] = f;
        head = next;
        return true;
    }

    bool pop(solrac::CanFdFrame &out) noexcept
    {
        if (tail == head)
        {
            return false;
        }
        out = frames[tail];
        tail = (tail + 1U) % kCapacity;
        return true;
    }
};

class MockPhy final : public solrac::ICanPhy
{
public:
    MockPhy(VirtualCanWire &tx_wire, VirtualCanWire &rx_wire) noexcept
        : tx_wire_(tx_wire), rx_wire_(rx_wire) {}

    bool init() noexcept override { return true; }
    bool transmit(const solrac::CanFdFrame &f) noexcept override
    {
        return healthy_ && tx_wire_.push(f);
    }
    bool receive(solrac::CanFdFrame &out) noexcept override
    {
        return healthy_ && rx_wire_.pop(out);
    }
    bool isHealthy() const noexcept override { return healthy_; }
    void setHealthy(bool h) noexcept { healthy_ = h; }

private:
    VirtualCanWire &tx_wire_;
    VirtualCanWire &rx_wire_;
    bool healthy_{true};
};

int main()
{
    using namespace vehicle_network_example;
    const std::size_t kTableLen = sizeof(kRoutingTable) / sizeof(kRoutingTable[0]);

    // ------------------------------------------------------------------------
    // TEST 1: 2-Bus Mode (TTC + GEN), GEN-only Mode, and TTC-only Mode
    // ------------------------------------------------------------------------
    VirtualCanWire ttc_ctrl_to_act, ttc_act_to_ctrl;
    VirtualCanWire gen_ctrl_to_act, gen_act_to_ctrl;
    VirtualCanWire gen_only_to_ctrl, ctrl_to_gen_only;
    VirtualCanWire ttc_only_to_ctrl, ctrl_to_ttc_only;

    MockPhy ctrl_ttc(ttc_ctrl_to_act, ttc_only_to_ctrl);
    MockPhy ctrl_gen(gen_ctrl_to_act, gen_only_to_ctrl);

    MockPhy act_ttc(ttc_act_to_ctrl, ttc_ctrl_to_act);
    MockPhy act_gen(gen_act_to_ctrl, gen_ctrl_to_act);

    // GEN-only node (TTC bus = nullptr)
    MockPhy gen_only_phy(gen_only_to_ctrl, ctrl_to_gen_only);

    // TTC-only node (GEN bus = nullptr)
    MockPhy ttc_only_phy(ttc_only_to_ctrl, ctrl_to_ttc_only);

    solrac::SolracBus ctrl_bus(kPrimaryControllerNode, &ctrl_ttc, &ctrl_gen);
    solrac::SolracBus act_bus(kActuatorNode, &act_ttc, &act_gen);
    solrac::SolracBus gen_only_bus(kGenOnlyNode, nullptr, &gen_only_phy);
    solrac::SolracBus ttc_only_bus(kTtcOnlyNode, &ttc_only_phy, nullptr);

    assert(ctrl_bus.init() == solrac::Status::kOk);
    assert(act_bus.init() == solrac::Status::kOk);
    assert(gen_only_bus.init() == solrac::Status::kOk);
    assert(ttc_only_bus.init() == solrac::Status::kOk);

    assert(ctrl_bus.configurePorts(kRoutingTable, kTableLen) == solrac::Status::kOk);
    assert(act_bus.configurePorts(kRoutingTable, kTableLen) == solrac::Status::kOk);
    assert(gen_only_bus.configurePorts(kRoutingTable, kTableLen) == solrac::Status::kOk);
    assert(ttc_only_bus.configurePorts(kRoutingTable, kTableLen) == solrac::Status::kOk);

    std::uint32_t critical_exec_count = 0U;
    std::uint32_t gen_only_rx_count = 0U;
    std::uint32_t ttc_only_rx_count = 0U;

    assert(act_bus.subscribeServiceRequest(
               kCriticalCommandPort,
               [](const solrac::TransferInfo &, const std::uint8_t *p, std::size_t n, void *ctx) noexcept
               {
                   CriticalCommandReq cmd{};
                   if (solrac::unpackPayload(p, n, cmd) && (cmd.arm_key == 0x5AU))
                   {
                       (*static_cast<std::uint32_t *>(ctx))++;
                   }
               },
               &critical_exec_count) == solrac::Status::kOk);

    assert(ctrl_bus.subscribeMessage(
               kGenOnlyStatusPort,
               [](const solrac::TransferInfo &, const std::uint8_t *p, std::size_t n, void *ctx) noexcept
               {
                   NodeStatusMsg msg{};
                   if (solrac::unpackPayload(p, n, msg) && (msg.status_code == 1U))
                   {
                       (*static_cast<std::uint32_t *>(ctx))++;
                   }
               },
               &gen_only_rx_count) == solrac::Status::kOk);

    assert(ctrl_bus.subscribeMessage(
               kTtcOnlyStatusPort,
               [](const solrac::TransferInfo &, const std::uint8_t *p, std::size_t n, void *ctx) noexcept
               {
                   NodeStatusMsg msg{};
                   if (solrac::unpackPayload(p, n, msg) && (msg.status_code == 2U))
                   {
                       (*static_cast<std::uint32_t *>(ctx))++;
                   }
               },
               &ttc_only_rx_count) == solrac::Status::kOk);

    // Dual-cast command -> sent on TTC + GEN, deduplicated to 1 execution
    CriticalCommandReq cmd{1U, 0x5AU};
    assert(ctrl_bus.sendRequest(kActuatorNode, kCriticalCommandPort, cmd, 1000ULL) ==
           solrac::Status::kOk);
    ctrl_bus.update(1000ULL);
    act_bus.update(1100ULL);
    assert(critical_exec_count == 1U);

    // GEN-only mode test
    NodeStatusMsg gen_status{1U, 4096U};
    assert(gen_only_bus.publish(kGenOnlyStatusPort, gen_status, 2000ULL) == solrac::Status::kOk);
    gen_only_bus.update(2000ULL);
    ctrl_bus.update(2100ULL);
    assert(gen_only_rx_count == 1U);

    gen_only_phy.setHealthy(false);
    assert(gen_only_bus.publish(kGenOnlyStatusPort, gen_status, 3000ULL) ==
           solrac::Status::kTargetBusUnavailable);

    // TTC-only mode test
    NodeStatusMsg ttc_status{2U, 100U};
    assert(ttc_only_bus.publish(kTtcOnlyStatusPort, ttc_status, 3500ULL) == solrac::Status::kOk);
    ttc_only_bus.update(3500ULL);
    ctrl_bus.update(3600ULL);
    assert(ttc_only_rx_count == 1U);

    ttc_only_phy.setHealthy(false);
    assert(ttc_only_bus.publish(kTtcOnlyStatusPort, ttc_status, 3800ULL) ==
           solrac::Status::kTargetBusUnavailable);

    // ------------------------------------------------------------------------
    // TEST 2: 1-Bus Mode
    // ------------------------------------------------------------------------
    VirtualCanWire bus1_a_to_b, bus1_b_to_a;
    MockPhy single_phy_a(bus1_a_to_b, bus1_b_to_a);
    MockPhy single_phy_b(bus1_b_to_a, bus1_a_to_b);

    solrac::SolracBus single_bus_node_a(1U, single_phy_a);
    solrac::SolracBus single_bus_node_b(2U, single_phy_b);
    assert(single_bus_node_a.isSingleBusMode());
    assert(single_bus_node_a.init() == solrac::Status::kOk);
    assert(single_bus_node_b.init() == solrac::Status::kOk);
    assert(single_bus_node_a.configurePorts(kRoutingTable, kTableLen) == solrac::Status::kOk);

    std::uint32_t single_bus_rx_count = 0U;
    assert(single_bus_node_b.subscribeMessage(
               kStateTelemPort,
               [](const solrac::TransferInfo &, const std::uint8_t *p, std::size_t n, void *ctx) noexcept
               {
                   StateTelemetryMsg st{};
                   if (solrac::unpackPayload(p, n, st) && (st.state == 2U))
                   {
                       (*static_cast<std::uint32_t *>(ctx))++;
                   }
               },
               &single_bus_rx_count) == solrac::Status::kOk);

    StateTelemetryMsg boost_state{2U, 1520.5F};
    assert(single_bus_node_a.publish(kStateTelemPort, boost_state, 4000ULL) == solrac::Status::kOk);
    single_bus_node_a.update(4000ULL);
    single_bus_node_b.update(4100ULL);
    assert(single_bus_rx_count == 1U);

    std::printf("SUCCESS: 2-Bus mode, GEN-only mode, TTC-only mode, and 1-Bus mode all passed!\n");
    return 0;
}