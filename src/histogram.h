// The Histogram class creates a histogram of values. It also provides an
// automatic bin size selection (and thus number of bins) using Scott's Rule.
// Other features include the ability to query the histogram for the bins that
// contain rare values.

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>


namespace iv_descriptor {

class Histogram {
public:
  // Uses Scott's Rule (1979) to determine the optimal histogram bin size given
  // the set of values.
  // Returns 0 if values is an empty list (where the bin size is undefined).
  static float ScottsRuleBinSize(const std::vector<float> &values);

  // Builds the histogram from the given list of values. The histogram will
  // preserve the index of the given values as given.
  Histogram(const std::vector<float> &values);

private:
  // This list of lists represents the bins of the histogram. Each index in the
  // outer list is the bin number. The list of numbers in each bin correspond
  // to the indices of the data points in that bin.
  //
  // TODO: might be faster to sort by bin size (using ordered set) for faster
  // lookup of bins with the fewest data points.
  std::vector<std::vector<int>> bins_;

};  // class Histogram

};  // namespace iv_descriptor


#endif  // HISTOGRAM_H
