// 15 segments, 1855 mm -- 1,1 -> 4,6, r=30 mm
// Robot frame: x forward, y left, mm; path starts at the robot's pose.
planner.appendSegment(Segment({0.00f, 0.00f}, {1.42f, 0.03f}, 1.0f / 30.00f, Segment::Direction::Left));
planner.appendSegment(Segment({1.42f, 0.03f}, {661.00f, 31.34f}));
planner.appendSegment(Segment({661.00f, 31.34f}, {685.61f, 46.41f}, 1.0f / 30.00f, Segment::Direction::Left));
planner.appendSegment(Segment({685.61f, 46.41f}, {689.79f, 52.06f}, 1.0f / 30.00f, Segment::Direction::Right));
planner.appendSegment(Segment({689.79f, 52.06f}, {791.31f, 160.13f}));
planner.appendSegment(Segment({791.31f, 160.13f}, {813.85f, 169.58f}, 1.0f / 30.00f, Segment::Direction::Right));
planner.appendSegment(Segment({813.85f, 169.58f}, {1128.47f, 162.44f}));
planner.appendSegment(Segment({1128.47f, 162.44f}, {1150.43f, 171.28f}, 1.0f / 30.00f, Segment::Direction::Left));
planner.appendSegment(Segment({1150.43f, 171.28f}, {1151.14f, 171.97f}, 1.0f / 30.00f, Segment::Direction::Right));
planner.appendSegment(Segment({1151.14f, 171.97f}, {1245.65f, 260.94f}));
planner.appendSegment(Segment({1245.65f, 260.94f}, {1233.36f, 311.62f}, 1.0f / 30.00f, Segment::Direction::Left));
planner.appendSegment(Segment({1233.36f, 311.62f}, {960.62f, 389.90f}));
planner.appendSegment(Segment({960.62f, 389.90f}, {940.23f, 409.90f}, 1.0f / 30.00f, Segment::Direction::Right));
planner.appendSegment(Segment({940.23f, 409.90f}, {900.95f, 537.35f}));
planner.appendSegment(Segment({900.95f, 537.35f}, {900.00f, 540.00f}, 1.0f / 30.00f, Segment::Direction::Left));
