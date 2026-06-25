#include <iostream>
#include <thread>
#include <chrono>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    std::string interface_name = "enp88s0";

    try {
        // 1. 채널 팩토리 초기화
        unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);

        // 2. SportClient 생성 및 초기화
        unitree::robot::go2::SportClient sport_client;
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        std::cout << "=== Start Forward/Backward Motion ===" << std::endl;

        // 먼저 로봇을 일어서게 합니다 (StandUp 상태여야 Move가 가능함)
        sport_client.RecoveryStand();
        std::this_thread::sleep_for(2s);

        // 3. 전진 (Forward)
        // Move(vx, vy, vyaw) -> vx: 0.5 m/s (전진)
        std::cout << "[Motion] Moving Forward (0.5 m/s)..." << std::endl;
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < 3s) {
            sport_client.Move(0.5f, 0.0f, 0.0f); 
            std::this_thread::sleep_for(50ms); // 20Hz
        }
        std::this_thread::sleep_for(1s);

        // 4. 정지 (Stop/Stand)
        // 속도를 모두 0으로 주면 제자리에서 균형을 잡으며 멈춥니다.
        std::cout << "[Motion] Stopping..." << std::endl;
        sport_client.Move(0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(3s);

        // 5. 후진 (Backward)
        // vx: -0.5 m/s (후진)
        std::cout << "[Motion] Moving Backward (-0.5 m/s)..." << std::endl;
        start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < 3s) {
            sport_client.Move(-0.5f, 0.0f, 0.0f);
            std::this_thread::sleep_for(50ms);
        }
        std::this_thread::sleep_for(1s);
        
        // 6. 오른쪽 주행
        // vy: -0.5 m/s
        std::cout << "[Motion] Moving Right (-0.5 m/s)..." << std::endl;
        start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < 3s) {
            sport_client.Move(0.0f, -0.5f, 0.0f);
            std::this_thread::sleep_for(50ms);
        }
        std::this_thread::sleep_for(1s);


        // 7. 최종 정지
        std::cout << "[Motion] Final Stop" << std::endl;
        sport_client.Move(0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(3s);

        // StandDown
        std::cout << "\n[Motion] Executing StandDown..." << std::endl;
        sport_client.StandDown();
        std::cout << "=== Motion Sequence Completed ===" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[Error] Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}