#include <iostream>
#include <thread>
#include <chrono>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    // enp88s0 인터페이스 사용 (고정)
    std::string interface_name = "enp88s0";

    std::cout << "[SDK Example] Initializing channel factory with interface: " 
              << interface_name << std::endl;

    try {
        // 1 채널 팩토리 초기화 (네트워크 설정)
        unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);
        std::cout << "[SDK Example] ChannelFactory initialized successfully!" << std::endl;

        // 2 SportClient 생성 및 초기화
        unitree::robot::go2::SportClient sport_client;
        sport_client.SetTimeout(10.0f);
        sport_client.Init();
        std::cout << "[SDK Example] SportClient initialized!" << std::endl;

        // 3 모션 시퀀스: 일어서기 → 기다리기 → 앉기
        std::cout << "\n=== Start Motion Sequence ===" << std::endl;

        // RECOVERY_STAND
        std::cout << "\n[Motion] Executing RecoveryStand..." << std::endl;
        int ret = sport_client.RecoveryStand();
        std::cout << "[Result] RecoveryStand() returned: " << ret << std::endl;
        std::this_thread::sleep_for(3s);

        // STAND_DOWN
        std::cout << "\n[Motion] Executing STAND_DOWN..." << std::endl;
        ret = sport_client.StandDown();
        std::cout << "[Result] StandDown() returned: " << ret << std::endl;
        std::this_thread::sleep_for(3s);

        // STAND_UP again
        std::cout << "\n[Motion] Executing STAND_UP again..." << std::endl;
        ret = sport_client.StandUp();
        std::cout << "[Result] StandUp() returned: " << ret << std::endl;
        std::this_thread::sleep_for(3s);

        // HELLO
        std::cout << "\n[Motion] Executing HELLO..." << std::endl;
        ret = sport_client.Hello();
        std::cout << "[Result] Hello() returned: " << ret << std::endl;
        std::this_thread::sleep_for(3s);

        // StandDown
        std::cout << "\n[Motion] Executing StandDown..." << std::endl;
        ret = sport_client.StandDown();
        std::cout << "[Result] StandDown() returned: " << ret << std::endl;
        std::this_thread::sleep_for(3s);

        std::cout << "\n=== Motion Sequence Completed ===" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[Error] Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
