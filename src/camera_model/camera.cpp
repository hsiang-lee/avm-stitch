// Camera value object (camera.hpp) is header-only: every member is a
// one-line delegate to the underlying CameraModel. This TU exists purely as
// a self-containment check — it compiles camera.hpp standalone, catching any
// missing-include regression in the header.
#include "camera_model/camera.hpp"
