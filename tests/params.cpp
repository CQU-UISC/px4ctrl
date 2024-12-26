#include <gtest/gtest.h>
#include <iostream>
#include <spdlog/spdlog.h>
#include <filesystem>
#include "params.h"

using namespace px4ctrl;
std::string file_path = __FILE__;
std::string ws = std::filesystem::path(file_path).parent_path().string();

TEST(px4ctrl, TestLoadParams){
    // file path in current working directory
    std::string file = ws + "/xi35.yaml";
    EXPECT_NO_THROW(Px4CtrlParams::load(file));
}

TEST(px4ctrl, TestPrintParams){
    // file path in current working directory
    std::string file = ws + "/xi35.yaml";
    Px4CtrlParams params = Px4CtrlParams::load(file);
    std::cout << params << std::endl;
}