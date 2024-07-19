#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "manager.hpp"
#include "mock_driver.hpp"

using ::testing::Exactly;
using ::testing::_;

TEST(ManagerTests, ManagerTest1)
{
    MockDriver myMockDriver;
    
    // first call flashes once
    EXPECT_CALL(myMockDriver, flashLED()).Times(Exactly(1));
    
    // second call flashes twice
    {
        ::testing::InSequence s;

        EXPECT_CALL(myMockDriver, flashLED()).Times(Exactly(1));
        EXPECT_CALL(myMockDriver, wait(_)).Times(Exactly(1));
        EXPECT_CALL(myMockDriver, flashLED()).Times(Exactly(1));
    }

    Manager myManager(&myMockDriver);
    for(int i = 0; i < 2; ++i)
    {
        myManager.runManager();
    }
}
