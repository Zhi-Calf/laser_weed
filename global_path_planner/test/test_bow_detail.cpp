// properlyCrosses 的严格穿越语义。

#include <gtest/gtest.h>

#include "bow_detail.hpp"

namespace {

Segment_2 Seg(double x1, double y1, double x2, double y2) {
  return {Point_2(x1, y1), Point_2(x2, y2)};
}

}  // namespace

TEST(ProperlyCrosses, RejectsDisjointSegments) {
  EXPECT_FALSE(bow_detail::properlyCrosses(Seg(0, 0, 1, 0), Seg(0, 1, 1, 1)));
}

TEST(ProperlyCrosses, AcceptsInteriorCrossing) {
  EXPECT_TRUE(bow_detail::properlyCrosses(Seg(0, 0, 2, 2), Seg(0, 2, 2, 0)));
  EXPECT_TRUE(bow_detail::properlyCrosses(Seg(0, 0, 4, 0), Seg(2, -1, 2, 1)));
}

TEST(ProperlyCrosses, RejectsSharedEndpoint) {
  EXPECT_FALSE(bow_detail::properlyCrosses(Seg(0, 0, 2, 0), Seg(2, 0, 2, 2)));
}

TEST(ProperlyCrosses, RejectsTJunction) {
  EXPECT_FALSE(bow_detail::properlyCrosses(Seg(0, 0, 2, 0), Seg(1, 0, 1, 2)));
}

TEST(ProperlyCrosses, RejectsCollinearOverlap) {
  EXPECT_FALSE(bow_detail::properlyCrosses(Seg(0, 0, 2, 0), Seg(1, 0, 3, 0)));
  EXPECT_FALSE(bow_detail::properlyCrosses(Seg(0, 0, 3, 0), Seg(1, 0, 2, 0)));
}

TEST(ClipLineToRegion, ClipsAndReversesSegment) {
  const Polygon_2 region{Point_2(0, 0), Point_2(10, 0), Point_2(10, 10), Point_2(0, 10)};
  const auto forward = bow_detail::clipLineToRegion(region, Point_2(-1, 5), Point_2(11, 5), Point_2(1, 0), false);
  ASSERT_EQ(forward.size(), 1u);
  EXPECT_NEAR((forward[0].first - Point_2(0, 5)).norm(), 0.0, 1e-9);
  EXPECT_NEAR((forward[0].second - Point_2(10, 5)).norm(), 0.0, 1e-9);

  const auto reverse = bow_detail::clipLineToRegion(region, Point_2(-1, 5), Point_2(11, 5), Point_2(1, 0), true);
  ASSERT_EQ(reverse.size(), 1u);
  EXPECT_NEAR((reverse[0].first - Point_2(10, 5)).norm(), 0.0, 1e-9);
  EXPECT_NEAR((reverse[0].second - Point_2(0, 5)).norm(), 0.0, 1e-9);
}

TEST(ClipLineToRegion, ReturnsEmptyWhenLineMissesRegion) {
  const Polygon_2 region{Point_2(0, 0), Point_2(10, 0), Point_2(10, 10), Point_2(0, 10)};
  EXPECT_TRUE(bow_detail::clipLineToRegion(region, Point_2(-1, 20), Point_2(11, 20), Point_2(1, 0), false).empty());
}

TEST(RowDistanceToSegment, UsesClosestRowSegment) {
  const std::vector<Segment_2> row{Seg(0, 0, 10, 0), Seg(0, 3, 10, 3)};
  EXPECT_NEAR(bow_detail::rowDistanceToSegment(row, Seg(0, 5, 10, 5)), 2.0, 1e-9);
}
