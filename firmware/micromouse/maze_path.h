// 16 segments, 896 mm -- 1,1 -> 5,2, r=25 mm
// Robot frame: x forward, y left, mm; path starts at the robot's pose.
planner.appendSegment(Segment({0.00f, 0.00f}, {24.31f, 30.83f}, 1.0f / 25.00f, Segment::Direction::Left));
planner.appendSegment(Segment({24.31f, 30.83f}, {14.33f, 72.47f}));
planner.appendSegment(Segment({14.33f, 72.47f}, {14.14f, 73.24f}, 1.0f / 25.00f, Segment::Direction::Left));
planner.appendSegment(Segment({14.14f, 73.24f}, {13.28f, 78.72f}, 1.0f / 25.00f, Segment::Direction::Right));
planner.appendSegment(Segment({13.28f, 78.72f}, {11.73f, 113.28f}));
planner.appendSegment(Segment({11.73f, 113.28f}, {17.74f, 131.13f}, 1.0f / 25.00f, Segment::Direction::Right));
planner.appendSegment(Segment({17.74f, 131.13f}, {53.93f, 173.19f}));
planner.appendSegment(Segment({53.93f, 173.19f}, {67.74f, 181.35f}, 1.0f / 25.00f, Segment::Direction::Right));
planner.appendSegment(Segment({67.74f, 181.35f}, {136.45f, 195.78f}));
planner.appendSegment(Segment({136.45f, 195.78f}, {155.97f, 216.12f}, 1.0f / 25.00f, Segment::Direction::Left));
planner.appendSegment(Segment({155.97f, 216.12f}, {158.09f, 222.82f}, 1.0f / 25.00f, Segment::Direction::Right));
planner.appendSegment(Segment({158.09f, 222.82f}, {302.04f, 522.44f}));
planner.appendSegment(Segment({302.04f, 522.44f}, {298.41f, 549.63f}, 1.0f / 25.00f, Segment::Direction::Left));
planner.appendSegment(Segment({298.41f, 549.63f}, {296.74f, 551.77f}, 1.0f / 25.00f, Segment::Direction::Right));
planner.appendSegment(Segment({296.74f, 551.77f}, {182.89f, 716.53f}));
planner.appendSegment(Segment({182.89f, 716.53f}, {180.00f, 720.00f}, 1.0f / 25.00f, Segment::Direction::Left));
