/*
Author: David Akre
Date: 4/16/24
Description: Header file to define some constants for demo
*/

namespace optical_flow {
	constexpr int NUM_CAMERAS_PER_PI = 1;
	constexpr int NUM_POINTS = 400;
	constexpr int NUM_RINGS = 10;
	constexpr int NUM_PIS = 1;
	constexpr int FLOW_THRESHOLD = 10;
	constexpr const char* TOPIC_NS = "rpi";
	constexpr const char* TOPIC = "optical_flow_vectors_";
	enum ProjectionScheme {
		RECTILINEAR,
		CONCENTRIC
	};
} // namespace optical_flow
