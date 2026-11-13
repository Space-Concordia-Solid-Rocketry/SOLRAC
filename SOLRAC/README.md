# SOLRAC — Solid Rocketry Avionics CAN-FD Transport Library

**SOLRAC** (**Sol**id **R**ocketry **A**vionics **C**AN) is a deterministic, **JSF AV C++**–compliant communication library built for STM32 avionics stacks. Powered by **OpenCyphal (`libcanard` v5.0 / Cyphal v1.1)** and the **$O(1)$ static memory allocator (`o1heap`)**, SOLRAC provides hardware-arbitrated message prioritization, multi-frame CAN-FD segmentation, automatic dual-bus deduplication, and seamless bus failover across **TTC** (Telemetry, Tracking & Command) and **GEN** (General) CAN-FD lines.

---

## Table of Contents
1. [Key Features](#1-key-features)
2. [Repository Architecture](#2-repository-architecture)
3. [Compliance & Operational Specification (RFC 2119)](#3-compliance--operational-specification-rfc-2119)
4. [Supported Bus Topologies](#4-supported-bus-topologies)
5. [Priority Levels & Routing Policies](#5-priority-levels--routing-policies)
6. [Installation (PlatformIO & STM32CubeIDE)](#6-installation-platformio--stm32cubeide)
7. [Defining a Vehicle Network (Shared ICD Pattern)](#7-defining-a-vehicle-network-shared-icd-pattern)
8. [Usage Examples](#8-usage-examples)
9. [Hardware Drivers & Power Gating (`TCAN1576DMTRQ1`)](#9-hardware-drivers--power-gating-tcan1576dmtrq1)
10. [Building & Running PC Unit Tests](#10-building--running-pc-unit-tests)

---

## 1. Key Features

* **Zero Flight Heap Allocation:** Uses a fixed 16 KiB static memory pool managed by `o1heap`. No `new`, `delete`, `malloc()`, or `free()` calls occur during flight.
* **Hardware-Arbitrated Priority (8 Levels):** Maps OpenCyphal's 3-bit priority field to the Most Significant Bits (`[28:26]`) of the 29-bit CAN-FD Extended Identifier so critical abort and actuation commands win bus arbitration with zero software latency.
* **Flexible Vehicle Topologies:** Supports **1-Bus Mode**, **2-Bus Mode** (`TTC` + `GEN`), **TTC-only Mode**, and **GEN-only Mode** using the exact same library binary.
* **Automatic Dual-Cast Deduplication:** Critical commands configured with `RoutePolicy::kDualCast` transmit simultaneously on both `TTC` and `GEN` with identical transfer-IDs. Receiving nodes execute the first arriving transfer and automatically discard the redundant copy.
* **Automatic Bus Failover:** Primary-routed telemetry (`kTtcPrimary` / `kGenPrimary`) detects controller `BUS_OFF` or error-passive states and fails over to the healthy bus automatically.
* **Declarative Routing Table:** Teams bind Port-IDs to priorities and bus policies once at startup via `configurePorts()`. Application code simply calls `publish()` or `sendRequest()`.
* **Bundled Hardware Drivers:** Includes hardware-abstracted drivers for STM32 internal **FDCAN** peripherals and the **TI TCAN1576DMTRQ1** CAN-FD SIC transceiver (including `INH` regulator power-gating for pad power savings).

---

## 2. Repository Architecture

```text
SOLRAC/
├── .vscode/
│   └── settings.json             # VS Code CMake & C++17 IntelliSense configuration
├── dependancies/                 # Vendored MISRA-C third-party dependencies
│   ├── libcanard/                # OpenCyphal Cyphal/CAN v1.1 transport engine (v5.0)
│   │   ├── canard.c
│   │   ├── canard.h
│   │   └── cavl2.h
│   └── o1heap/                   # Deterministic O(1) constant-time static allocator
│       ├── o1heap.c
│       └── o1heap.h
├── drivers/                      # Hardware Abstraction Layer (HAL-independent interfaces)
│   ├── inc/
│   │   ├── can_phy.hpp           # Abstract ICanPhy interface & CanFdFrame struct
│   │   ├── stm32_fdcan_phy.hpp   # STM32 internal FDCAN controller driver interface
│   │   └── tcan1576_phy.hpp      # TI TCAN1576DMTRQ1 transceiver & INH power-gating driver
│   └── src/
│       ├── stm32_fdcan_phy.cpp   # STM32 HAL FDCAN implementation (guarded for portability)
│       └── tcan1576_phy.cpp      # TCAN1576 SPI Mode 0 state machine & fault handler
├── inc/
│   └── solrac.hpp                # Public C++17 library API (only header apps include)
├── src/
│   └── solrac.cpp                # Core dual-bus routing, failover, & libcanard integration
├── test/
│   └── test_dual_bus.cpp         # Native PC simulation suite (1-Bus, 2-Bus, TTC/GEN-only)
├── .gitignore
├── CMakeLists.txt                # CMake build script (STM32Cube CMake + PC unit tests)
├── library.json                  # PlatformIO library manifest
└── README.md
```

---

## 3. Compliance & Operational Specification (RFC 2119)

The key words **"MUST"**, **"MUST NOT"**, **"REQUIRED"**, **"SHALL"**, **"SHALL NOT"**, **"SHOULD"**, **"SHOULD NOT"**, **"RECOMMENDED"**, **"MAY"**, and **"OPTIONAL"** in this documentation are to be interpreted as described in **RFC 2119**.

### JSF AV C++ Coding Standard Compliance
* **Dynamic Memory (JSF AV Rule 206):** `SOLRAC` **MUST NOT** allocate memory from the system heap (`new`/`malloc`). All internal queues and reassembly buffers **SHALL** be allocated from the static `arena_` buffer inside `SolracBus`.
* **Exceptions & RTTI (JSF AV Rule 208):** `SOLRAC` **MUST** compile with `-fno-exceptions` and `-fno-rtti`. All fallible operations **SHALL** return a deterministic `solrac::Status` code or `bool`.
* **Type Safety (JSF AV Rules 185 & 209):** Public interfaces **SHALL** use fixed-width integer types (`std::uint8_t`, `std::uint16_t`, `std::uint32_t`, `std::uint64_t`), `nullptr`, and explicit C++ casts (`static_cast`).
* **Message Struct Constraints:** All user-defined payload structs passed to `publish()`, `sendRequest()`, `sendResponse()`, and `unpackPayload()` **MUST** satisfy `std::is_trivially_copyable_v<T>` and `std::is_standard_layout_v<T>` (enforced via `static_assert` at compile time).

### Runtime Lifecycle Contract
1. **Hardware Initialization:** Firmware **MUST** call `phy.init()` on each active `ICanPhy` instance before calling `bus.init()`.
2. **Bus Initialization:** Firmware **MUST** call `bus.init()` once at startup before registering routes, subscribing to ports, or calling `update()`.
3. **Deterministic Polling:** Firmware **MUST** call `bus.update(now_usec)` periodically in its main superloop or RTOS communication task with a monotonically increasing microsecond timestamp (`now_usec`).
4. **Non-Reentrant Callbacks:** Subscription callbacks **MUST NOT** block, perform long I/O delays, or cut board power directly inside the callback context.

---

## 4. Supported Bus Topologies

`SolracBus` adapts to any vehicle or board wiring configuration via its two constructors:

| Topology Mode | Constructor Call | Behavior |
| :--- | :--- | :--- |
| **2-Bus Mode (`TTC` + `GEN`)** | `SolracBus(node_id, &ttc_phy, &gen_phy)` | Full dual-redundant operation across Bus 0 (`TTC`) and Bus 1 (`GEN`). Supports `kDualCast`, primary/failover, and bus-isolated traffic. |
| **GEN-only Mode** | `SolracBus(node_id, nullptr, &gen_phy)` | For non-critical nodes wired exclusively to `GEN`. Bus 0 (`TTC`) is disabled; all TX/RX occurs strictly on `GEN`. |
| **TTC-only Mode** | `SolracBus(node_id, &ttc_phy, nullptr)` | For nodes wired exclusively to `TTC`. Bus 1 (`GEN`) is disabled; all TX/RX occurs strictly on `TTC`. |
| **1-Bus Mode** | `SolracBus(node_id, single_phy)` | For single-bus vehicles. All routing policies automatically route over `single_phy`. |

---

## 5. Priority Levels & Routing Policies

### Hardware Priority Levels (`solrac::Priority`)
Lower numeric values occupy lower bit values in bits `[28:26]` of the 29-bit CAN-FD identifier and win physical bus arbitration first:

| Enum Value | Numeric ID | Intended Traffic Class |
| :--- | :---: | :--- |
| `Priority::kExceptional` | `0` | Flight termination, abort, critical safety interlocks |
| `Priority::kImmediate` | `1` | Pyro firing, main propulsion valve actuation |
| `Priority::kFast` | `2` | High-rate guidance, navigation, and control (GNC) commands |
| `Priority::kHigh` | `3` | Real-time RF downlink telemetry frames |
| `Priority::kNominal` | `4` | Standard sensor and battery/power telemetry *(default)* |
| `Priority::kLow` | `5` | Slow housekeeping (temperatures, peripheral status flags) |
| `Priority::kSlow` | `6` | Background diagnostics |
| `Priority::kOptional` | `7` | Bulk log dumps and non-critical debug traffic |

### Routing Policies (`solrac::RoutePolicy`)

| Policy Enum | Primary Bus | Failover / Redundancy Behavior |
| :--- | :---: | :--- |
| `RoutePolicy::kDualCast` | `TTC` + `GEN` | **MUST** transmit simultaneously on both `TTC` and `GEN`. Receiver automatically drops the duplicate frame. |
| `RoutePolicy::kTtcPrimary` | `TTC` (Bus 0) | **SHOULD** transmit on `TTC`; **MUST** fail over to `GEN` if `TTC` reports `!isHealthy()`. |
| `RoutePolicy::kGenPrimary` | `GEN` (Bus 1) | **SHOULD** transmit on `GEN`; **MUST** fail over to `TTC` if `GEN` reports `!isHealthy()`. |
| `RoutePolicy::kTtcOnly` | `TTC` (Bus 0) | **MUST** transmit only on `TTC`; **SHALL NOT** fail over to `GEN`. |
| `RoutePolicy::kGenOnly` | `GEN` (Bus 1) | **MUST** transmit only on `GEN`; **SHALL NOT** fail over to `TTC`. |

---

## 6. Installation (PlatformIO & STM32CubeIDE)

### Option A: PlatformIO (`platformio.ini`)
Add the GitHub repository URL to `lib_deps` in your board's `platformio.ini`. PlatformIO automatically reads `library.json` and compiles `src/`, `drivers/src/`, and `dependancies/`:

```ini
[env:avionics_board]
platform = ststm32
board = ...
framework = stm32cube
build_flags =
    -std=gnu++17
    -fno-exceptions
    -fno-rtti
lib_deps =
    [https://github.com/](https://github.com/)<your-org>/SOLRAC.git
```

### Option B: STM32CubeIDE (Managed C++ Build)
1. Clone `SOLRAC` into your project's `Middlewares/Third_Party/SOLRAC` directory.
2. Convert the project to C++ if generated as C (**Right-click Project $\rightarrow$ Convert to C++**).
3. Open **Project Properties $\rightarrow$ C/C++ General $\rightarrow$ Settings**:
   * Under **MCU G++ Compiler $\rightarrow$ Dialect**, select **ISO C++17 (`-std=c++17`)** and disable exceptions/RTTI (`-fno-exceptions -fno-rtti`).
   * Under **C/C++ General $\rightarrow$ Paths and Symbols $\rightarrow$ Includes**, add these 4 paths to both **GNU C** and **GNU C++**:
     * `Middlewares/Third_Party/SOLRAC/inc`
     * `Middlewares/Third_Party/SOLRAC/drivers/inc`
     * `Middlewares/Third_Party/SOLRAC/dependancies/libcanard`
     * `Middlewares/Third_Party/SOLRAC/dependancies/o1heap`

### Option C: STM32CubeMX CMake Project
Add `SOLRAC` as a subdirectory in your root `CMakeLists.txt`:
```cmake
add_subdirectory(Middlewares/Third_Party/SOLRAC)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE solrac)
```

---

## 7. Defining a Vehicle Network (Shared ICD Pattern)

`SOLRAC` deliberately **does not** hardcode board addresses, Port-IDs, or message structs so the library remains reusable across vehicles and modular as hardware evolves.

Create a single shared header (e.g., `vehicle_network.hpp` in a shared vehicle ICD repository) that all boards on your rocket `#include`:

```cpp
#ifndef VEHICLE_NETWORK_HPP
#define VEHICLE_NETWORK_HPP

#include <cstddef>
#include <cstdint>
#include "solrac.hpp"

namespace vehicle_net {

// 1. Unique Node-IDs per board (valid range: 1..127)
inline constexpr solrac::NodeId kFlightComputerNode  = 10U;
inline constexpr solrac::NodeId kActuatorNode        = 20U;
inline constexpr solrac::NodeId kPowerBoardNode      = 30U;
inline constexpr solrac::NodeId kRfTelemetryNode     = 40U;
inline constexpr solrac::NodeId kPeripheralGenNode   = 50U;

// 2. Port-IDs (Broadcast Subjects: 0..8191, Unicast RPC Services: 0..511)
inline constexpr solrac::PortId kFlightStateSubject  = 100U;
inline constexpr solrac::PortId kPowerStatusSubject  = 101U;
inline constexpr solrac::PortId kPeriphStatusSubject = 102U;
inline constexpr solrac::PortId kActuateService      = 200U;
inline constexpr solrac::PortId kPowerControlService = 210U;

// 3. Vehicle Routing Table (binds every Port-ID to Priority and Bus Policy)
inline constexpr solrac::PortConfig kRoutingTable[] = {
    {kActuateService,      solrac::Priority::kImmediate, solrac::RoutePolicy::kDualCast},
    {kFlightStateSubject,  solrac::Priority::kHigh,      solrac::RoutePolicy::kTtcPrimary},
    {kPowerStatusSubject,  solrac::Priority::kNominal,   solrac::RoutePolicy::kGenPrimary},
    {kPeriphStatusSubject, solrac::Priority::kLow,       solrac::RoutePolicy::kGenOnly},
    {kPowerControlService, solrac::Priority::kLow,       solrac::RoutePolicy::kGenOnly}
};

inline constexpr std::size_t kRoutingTableCount =
    sizeof(kRoutingTable) / sizeof(kRoutingTable[0]);

// 4. Packed Binary Payload Structs
struct __attribute__((packed)) PowerStatusMsg {
    std::uint32_t timestamp_ms;
    float         main_batt_v;
    float         aux_batt_v;
    std::uint8_t  fault_flags;
};

struct __attribute__((packed)) ActuateReq {
    std::uint8_t  channel_id;
    std::uint8_t  arm_key;      // Interlock byte (e.g., 0x5A)
    std::uint16_t duration_ms;
};

struct __attribute__((packed)) ActuateResp {
    std::uint8_t channel_id;
    std::uint8_t status;        // 0 = OK, 1 = REJECTED
};

struct __attribute__((packed)) PowerControlReq {
    std::uint8_t command;       // 0 = ENTER_SLEEP
    std::uint8_t magic_key;     // 0xA5
};

struct __attribute__((packed)) PowerControlResp {
    std::uint8_t accepted;      // 1 = ACK
};

}  // namespace vehicle_net

#endif  // VEHICLE_NETWORK_HPP
```

---

## 8. Usage Examples

### Example A: 2-Bus Controller Node (Pub/Sub + Dual-Cast RPC Request)

```cpp
#include "solrac.hpp"
#include "stm32_fdcan_phy.hpp"
#include "vehicle_network.hpp"

extern FDCAN_HandleTypeDef hfdcan1;  // TTC
extern FDCAN_HandleTypeDef hfdcan2;  // GEN

static solrac::Stm32FdcanPhy ttc_phy(hfdcan1);
static solrac::Stm32FdcanPhy gen_phy(hfdcan2);
static solrac::SolracBus     g_bus(vehicle_net::kFlightComputerNode, &ttc_phy, &gen_phy);

static void onPowerStatus(const solrac::TransferInfo& info,
                          const std::uint8_t* payload,
                          std::size_t payload_size,
                          void* /*user_ctx*/) noexcept {
    vehicle_net::PowerStatusMsg msg{};
    if (solrac::unpackPayload(payload, payload_size, msg)) {
        // Process battery voltages from info.source_node
        (void)info;
    }
}

static void onActuateResponse(const solrac::TransferInfo& info,
                              const std::uint8_t* payload,
                              std::size_t payload_size,
                              void* /*user_ctx*/) noexcept {
    vehicle_net::ActuateResp resp{};
    if (solrac::unpackPayload(payload, payload_size, resp)) {
        // Confirm execution from info.source_node
        (void)info;
    }
}

void controller_init() noexcept {
    (void)ttc_phy.init();
    (void)gen_phy.init();
    (void)g_bus.init();
    (void)g_bus.configurePorts(vehicle_net::kRoutingTable, vehicle_net::kRoutingTableCount);

    (void)g_bus.subscribeMessage(vehicle_net::kPowerStatusSubject, &onPowerStatus);
    (void)g_bus.subscribeServiceResponse(vehicle_net::kActuateService, &onActuateResponse);
}

void controller_loop(std::uint64_t now_usec) noexcept {
    // Automatically dual-casts on TTC + GEN at Priority::kImmediate
    const vehicle_net::ActuateReq req{1U, 0x5AU, 500U};
    (void)g_bus.sendRequest(vehicle_net::kActuatorNode,
                            vehicle_net::kActuateService,
                            req,
                            now_usec);

    g_bus.update(now_usec);
}
```

### Example B: GEN-Only Peripheral Node with `TCAN1576` Power Gating

On peripheral boards wired only to `GEN` where the `TCAN1576DMTRQ1` `INH` pin controls the board's voltage regulator enable (`EN`):

```cpp
#include "main.h"
#include "solrac.hpp"
#include "stm32_fdcan_phy.hpp"
#include "tcan1576_phy.hpp"
#include "vehicle_network.hpp"

// 1. Implement 16-bit SPI Mode 0 transfer for TCAN1576
class BoardTcanSpi final : public solrac::ITcan1576Spi {
public:
    BoardTcanSpi(SPI_HandleTypeDef& hspi, GPIO_TypeDef* cs_port, std::uint16_t cs_pin) noexcept
        : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin) {}

    bool transfer16(std::uint16_t tx_word, std::uint16_t& rx_word) noexcept override {
        const std::uint8_t tx[2] = {
            static_cast<std::uint8_t>((tx_word >> 8U) & 0xFFU),
            static_cast<std::uint8_t>(tx_word & 0xFFU)
        };
        std::uint8_t rx[2] = {0U, 0U};

        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        const HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi_, tx, rx, 2U, 5U);
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);

        rx_word = static_cast<std::uint16_t>((static_cast<std::uint16_t>(rx[0]) << 8U) | rx[1]);
        return st == HAL_OK;
    }

private:
    SPI_HandleTypeDef& hspi_;
    GPIO_TypeDef*      cs_port_;
    std::uint16_t      cs_pin_;
};

extern FDCAN_HandleTypeDef hfdcan1;
extern SPI_HandleTypeDef   hspi1;

static BoardTcanSpi          g_gen_spi(hspi1, CAN_CS_GPIO_Port, CAN_CS_Pin);
static solrac::Stm32FdcanPhy g_gen_ctrl(hfdcan1);
static solrac::Tcan1576Phy   g_gen_phy(g_gen_ctrl, g_gen_spi, /*enable_partial_nw=*/false);

// GEN-only mode: TTC bus pointer is nullptr
static solrac::SolracBus     g_bus(vehicle_net::kPeripheralGenNode, nullptr, &g_gen_phy);
static volatile bool         g_sleep_requested = false;

static void onPowerControlReq(const solrac::TransferInfo& info,
                              const std::uint8_t* payload,
                              std::size_t payload_size,
                              void* /*user_ctx*/) noexcept {
    vehicle_net::PowerControlReq req{};
    if (!solrac::unpackPayload(payload, payload_size, req)) {
        return;
    }

    vehicle_net::PowerControlResp resp{0U};
    if ((req.command == 0U) && (req.magic_key == 0xA5U)) {
        resp.accepted     = 1U;
        g_sleep_requested = true;
    }

    (void)g_bus.sendResponse(info.source_node,
                             info.port_id,
                             info.transfer_id,
                             resp,
                             info.timestamp_usec);
}

void peripheral_init() noexcept {
    // Clears POR interrupt flags and latches TCAN1576 into Normal Mode (keeps INH HIGH)
    (void)g_gen_phy.init();
    (void)g_bus.init();
    (void)g_bus.configurePorts(vehicle_net::kRoutingTable, vehicle_net::kRoutingTableCount);
    (void)g_bus.subscribeServiceRequest(vehicle_net::kPowerControlService, &onPowerControlReq);
}

void peripheral_loop(std::uint64_t now_usec) noexcept {
    g_bus.update(now_usec);

    if (g_sleep_requested) {
        // 1. Flush ACK response into FDCAN TX FIFO and allow frame transmission to complete
        g_bus.update(now_usec + 100ULL);
        HAL_Delay(2U);

        // 2. Drive TCAN1576 INH pin LOW -> cuts board regulator power until CAN bus wake
        (void)g_gen_phy.enterPowerDownSleep();
    }
}
```

### Example C: 1-Bus Mode Node

```cpp
static solrac::Stm32FdcanPhy single_phy(hfdcan1);
static solrac::SolracBus     g_bus(10U, single_phy);  // Automatically operates in 1-Bus Mode
```

---

## 9. Hardware Drivers & Power Gating (`TCAN1576DMTRQ1`)

### Built-In Drivers
1. **`solrac::Stm32FdcanPhy` (`drivers/inc/stm32_fdcan_phy.hpp`):**
   * Interfaces with STM32 internal FDCAN peripherals (`HAL_FDCAN_MODULE_ENABLED`).
   * Configures 29-bit extended ID acceptance into `RX_FIFO0`, transmits 64-byte CAN-FD frames with Bit Rate Switching (`BRS_ON`), and monitors `BusOff` / `ErrorPassive` protocol flags for `isHealthy()`.
   * **CubeMX Requirement:** Integrators **MUST** configure FDCAN nominal and data bit timings (e.g., 1 Mbps nominal / 5 Mbps data) in STM32CubeMX prior to calling `init()`.
2. **`solrac::Tcan1576Phy` (`drivers/inc/tcan1576_phy.hpp`):**
   * Wraps any `ICanPhy` controller alongside a 16-bit SPI Mode 0 (`CPOL=0, CPHA=0, <= 4 MHz`) `ITcan1576Spi` interface.
   * **`init()`:** Clears startup Power-On Reset (`POR`) and undervoltage interrupt registers (`0x51`–`0x54`), initializes the underlying FDCAN controller, switches `MODE_CNTRL` (`0x10`) to `Tcan1576Mode::kNormal` (`0x07`), and verifies the mode transition via SPI readback.
   * **`enterPowerDownSleep()`:** Clears wake interrupt flags and writes `Tcan1576Mode::kSleep` (`0x01`) to `MODE_CNTRL`, driving **`INH` LOW** to disable the downstream voltage regulator until a CAN wake-up event occurs.

### Adding a New CAN-FD Chip Driver
To support an external SPI CAN-FD controller (e.g., MCP2518FD or TCAN4550):
1. Create `drivers/inc/<chip>_phy.hpp` inheriting from `solrac::ICanPhy`.
2. Implement `init()`, `transmit()`, `receive()`, and `isHealthy()` in `drivers/src/<chip>_phy.cpp` without dynamic memory or exceptions.
3. Guard any vendor-specific HAL includes with `#if defined(...)` so native PC builds remain portable.

---

## 10. Building & Running PC Unit Tests

`SOLRAC` includes a native PC simulation harness (`test/test_dual_bus.cpp`) that verifies **2-Bus Mode** (dual-cast deduplication and failover), **GEN-only Mode**, **TTC-only Mode**, and **1-Bus Mode** under strict JSF compiler flags (`-fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wold-style-cast`).

```bash
# Configure and build the static library and unit test executable
cmake -B build
cmake --build build

# Run the simulation test suite
./build/solrac_tests
```