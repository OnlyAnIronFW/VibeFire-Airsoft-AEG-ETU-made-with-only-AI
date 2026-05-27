#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <string>

#include "autotrigger/yolo_infer.h"
#include "autotrigger/kalman_filter.h"
#include "autotrigger/ballistics.h"
#include "autotrigger/ranging.h"
#include "autotrigger/display.h"
#include "autotrigger/trigger.h"
#include "autotrigger/safety.h"
#include "autotrigger/pipeline.h"

static std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
    // NOTE: do NOT call std::cout here — not async-signal-safe
}

int main(int argc, char* argv[]) {
    std::cout << "AutoTrigger v1.0" << std::endl;
    std::cout << "Target: " <<
#ifdef MOCK_MODE
        "x86_64 (mock mode)"
#else
        "aarch64 (RK3566)"
#endif
        << std::endl;

    // ── Parse CLI arguments ────────────────────────
    std::string model_path = "models/yolov5n.rknn";
    std::string table_path = "tables/drop_table.bin";
    std::string uart_path = "/dev/ttyS1";
    float v0 = 70.0f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--model" && i + 1 < argc) model_path = argv[++i];
        else if (arg == "--table" && i + 1 < argc) table_path = argv[++i];
        else if (arg == "--uart" && i + 1 < argc) uart_path = argv[++i];
        else if (arg == "--v0" && i + 1 < argc) v0 = std::stof(argv[++i]);
        else {
            std::cerr << "Usage: autotrigger [--model PATH] [--table PATH] [--uart DEV] [--v0 MPS]\n";
            return 1;
        }
    }

    // ── Signal handling ────────────────────────────
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── Initialize modules ────────────────────────
    autotrigger::DisplayMock display;
    if (!display.init()) {
        std::cerr << "Failed to initialize display\n";
        return 1;
    }

    autotrigger::Ranging ranging(uart_path);
    autotrigger::Trigger trigger;
    autotrigger::Safety safety(nullptr, &ranging, &trigger);
    YOLOMock yolo; // TODO: replace with real YOLOInfer on ARM64
    KalmanFilter kalman;
    autotrigger::Ballistics ballistics;

    // Load ballistics table
    if (!ballistics.load_table(table_path)) {
        std::cout << "Drop table not found, using analytic fallback\n";
    } else {
        std::cout << "Drop table loaded: " << table_path << "\n";
    }

    // Initialize YOLO
    if (!yolo.init(model_path)) {
        std::cerr << "Failed to load model: " << model_path << "\n";
        return 1;
    }

    // Trigger must be initialized
    if (!trigger.init()) {
        std::cerr << "Failed to initialize trigger GPIO\n";
        return 1;
    }
    trigger.fire(false); // Safety: ensure LOW at startup

    // Safety startup check
    auto startup = safety.do_startup_check();
    if (!startup.can_proceed) {
        std::cerr << "Startup check failed\n";
        return 1;
    }
    if (startup.degraded) {
        std::cout << "Warning: started in degraded mode (ToF not responding)\n";
    }

    // ── Pipeline ───────────────────────────────────
    autotrigger::Pipeline pipeline(&yolo, &kalman, &ballistics,
                                    &ranging, &display, &trigger, &safety);
    if (!pipeline.init()) {
        std::cerr << "Failed to initialize pipeline\n";
        return 1;
    }

    // ── Main loop ──────────────────────────────────
    std::cout << "Running at " << (1.0f / 0.033f) << " FPS target...\n";
    auto last_frame = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<float>(now - last_frame).count();

        if (elapsed >= 0.033f) {
            // Static buffer — avoids 1.2 MB stack allocation on embedded
            static uint8_t dummy_rgb[640 * 640 * 3] = {};

            // ══ Safety runtime monitor (thermal + heartbeat + ToF recovery) ══
            safety.run_once();
            safety.kick_watchdog();

            // Run pipeline (computes aimpoint + updates trigger internally)
            pipeline.run_frame(dummy_rgb);

            // ── Post-pipeline watchdog kick (covers YOLO infer + ballistic compute) ──
            safety.kick_watchdog();

            // ── Safety gate: veto trigger output if safety conditions violated ──
            // Even if the pipeline wants to fire, safety has final authority.
            if (!safety.is_safe()) {
                trigger.fire(false);
            }

            last_frame = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // ── Graceful shutdown ─────────────────────────
    std::cout << "\nShutting down..." << std::endl;
    trigger.fire(false);
    display.release();
    std::cout << "AutoTrigger v1.0 stopped." << std::endl;
    return 0;
}
