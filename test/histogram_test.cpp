#include <vector>

#include "histogram.cpp"

#include <gtest/gtest.h>


TEST(HistogramTest, ExampleTest) {
  std::vector<float> vals = {0.1, 0.5, 1.2};
  iv_descriptor::Histogram h(vals);
  EXPECT_EQ(1, h.GetRareValues(0.1).size());
}
