# Normalized horizontal centroid error — positive = object right of center
float32 ex             # range [-1.0, 1.0]

# Detected contour area in pixels^2 — 0 when not detected
float32 area

# True when a valid target contour was found in the latest frame
bool object_detected
