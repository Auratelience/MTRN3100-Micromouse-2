// 13 segments, 488 mm -- 1,1 -> 3,2, r=30 mm
// Robot frame: x forward, y left, mm; path starts at the robot's pose.
planner.appendSegment(Segment({0.00f, 0.00f}, {26.37f, 44.30f}, 1.0f / 30.00f, Segment::Direction::Left));
planner.appendSegment(Segment({26.37f, 44.30f}, {17.05f, 61.50f}));
planner.appendSegment(Segment({17.05f, 61.50f}, {13.44f, 74.62f}, 1.0f / 30.00f, Segment::Direction::Right));
planner.appendSegment(Segment({13.44f, 74.62f}, {10.39f, 152.15f}));
planner.appendSegment(Segment({10.39f, 152.15f}, {41.84f, 183.29f}, 1.0f / 30.00f, Segment::Direction::Right));
planner.appendSegment(Segment({41.84f, 183.29f}, {45.73f, 183.36f}, 1.0f / 30.00f, Segment::Direction::Left));
planner.appendSegment(Segment({45.73f, 183.36f}, {110.90f, 188.64f}));
planner.appendSegment(Segment({110.90f, 188.64f}, {135.01f, 204.53f}, 1.0f / 30.00f, Segment::Direction::Left));
planner.appendSegment(Segment({135.01f, 204.53f}, {175.07f, 280.37f}));
planner.appendSegment(Segment({175.07f, 280.37f}, {178.02f, 299.95f}, 1.0f / 30.00f, Segment::Direction::Left));
planner.appendSegment(Segment({178.02f, 299.95f}, {177.53f, 306.94f}, 1.0f / 30.00f, Segment::Direction::Right));
planner.appendSegment(Segment({177.53f, 306.94f}, {179.97f, 358.59f}));
planner.appendSegment(Segment({179.97f, 358.59f}, {180.00f, 360.00f}, 1.0f / 30.00f, Segment::Direction::Left));
