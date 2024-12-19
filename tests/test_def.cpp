#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include "px4ctrl_def.h"
using namespace px4ctrl;

TEST(px4ctrl, TestObserver){
    auto data = std::make_shared<Px4Data<int>>();
    int result = 0;
    int result2 = 0;
    auto observer = data->observe([&result](const int& value){
        result = value;
    }); 
    auto observer2 = data->observe([&result2](const int& value){
        result2 = value;
    });
    data->post(10);
    EXPECT_EQ(data->value(), 10);
    EXPECT_EQ(result, 10);
    EXPECT_EQ(result2, 10);
    data->post(20);
    EXPECT_EQ(result, 20);
    EXPECT_EQ(result2, 20);
    observer2.reset();
    data->post(30);
    EXPECT_EQ(result, 30);
    EXPECT_EQ(result2, 20);
}