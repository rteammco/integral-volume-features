#include <vector>

#include "histogram.cpp"

#include <gtest/gtest.h>
#include <gmock/gmock.h>


using testing::IsEmpty;

TEST(HistogramTest, ExampleTest) {
  std::vector<float> vals = {0.1, 0.5, 1.2};
  iv_descriptor::Histogram h(vals);
  EXPECT_EQ(1, h.GetRareValues(0.1).size());
  EXPECT_EQ(3, h.GetRareValues(0.5).size());
  EXPECT_THAT(h.GetRareValues(0), IsEmpty());
}
