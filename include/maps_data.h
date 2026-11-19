#pragma once
#include <stdint.h>

// Baked-in fallback map data - INTENTIONALLY EMPTY in this
// repository. The firmware generates its own colour map on the
// device (GSI pale tiles, matched to the centre + receive range
// set in the setup portal) and stores it in the "mapdata" flash
// partition; see README "地図について". Until a map has been
// generated, the map area stays blank.
#define MAP_W 640
#define MAP_H 480
#define MAP_ROW_BYTES 320

static const uint8_t MAP_GRAY[2][153600] = {};
static const uint8_t MAP_CLASS[2][38400] = {};
