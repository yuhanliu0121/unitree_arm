#include <unitree/robot/channel/channel_publisher.hpp>

#include "msg/ArmString_.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

constexpr char kCommandTopic[] = "rt/arm_Command";
constexpr char kUnloadCommand[] =
    "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";

void PrintUsage(const char* program)
{
    std::cout
        << "Usage: " << program << " UNLOAD_ALL [interface]\n"
        << "Default interface: eno1\n"
        << "WARNING: This releases every arm joint. Support the arm before use.\n"
        << "An active real_bringup is stopped automatically.\n";
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        if (argc == 2 &&
            (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help"))
        {
            PrintUsage(argv[0]);
            return 0;
        }
        if (argc < 2 || argc > 3)
        {
            PrintUsage(argv[0]);
            return 1;
        }
        if (std::string(argv[1]) != "UNLOAD_ALL")
        {
            throw std::runtime_error("Explicit confirmation UNLOAD_ALL is required");
        }

        const std::string interface = argc == 3 ? argv[2] : "eno1";
        std::cout
            << "WARNING: Publishing global unload on interface '" << interface
            << "'. Every joint may immediately lose holding torque.\n";

        unitree::robot::ChannelFactory::Instance()->Init(0, interface);
        unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>
            publisher(kCommandTopic);
        publisher.InitChannel();
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        unitree_arm::msg::dds_::ArmString_ command{};
        command.data_() = kUnloadCommand;
        if (!publisher.Write(command))
        {
            throw std::runtime_error("DDS publisher.Write returned false");
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Sent: " << kUnloadCommand << '\n'
                  << "The controller status field cannot prove that every joint"
                     " unloaded; verify the physical arm while supporting it.\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "ERROR: " << error.what() << "\n\n";
        PrintUsage(argv[0]);
        return 1;
    }
}
