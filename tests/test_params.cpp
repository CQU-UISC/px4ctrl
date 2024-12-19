#include <gtest/gtest.h>
#include <iostream>
#include <spdlog/spdlog.h>
#include "px4ctrl_params.h"
#include <filesystem>

using namespace px4ctrl;

TEST(px4ctrl, TestLoadParams){
    // file path in current working directory
    std::string ws = std::filesystem::current_path().string();
    std::string file = ws + "/../config/xi35.yaml";
    EXPECT_NO_THROW(Px4CtrlParams::load(file));
}

TEST(px4ctrl, TestPrintParams){
    // file path in current working directory
    std::string ws = std::filesystem::current_path().string();
    std::string file = ws + "/../config/xi35.yaml";
    Px4CtrlParams params = Px4CtrlParams::load(file);
    std::cout << params << std::endl;
}