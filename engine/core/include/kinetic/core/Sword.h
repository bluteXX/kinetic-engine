#pragma once

#include "kinetic/core/Image.h"
#include "kinetic/core/MeshData.h"

namespace kinetic {

// A procedural longsword (hand-and-a-half), in metres: the grip's centre at the origin (where the
// fist holds it), the blade along +Y, its edges along +-X and its flats facing +-Z - the space of
// scene::WeaponComponent, whose default collision box covers this blade (y 0.10 .. 1.04).
//
//   pommel  wheel, y -0.172 .. -0.112
//   grip    leather over a barrel-shaped core with a spiral cord wrap, y -0.105 .. 0.104
//   guard   cross-guard curving slightly towards the blade, flared ends, 22 cm wide
//   blade   diamond section with a fuller over its first 60 %, bevelled edges, tapering to a
//           convex point at y 1.05
//
// Uses uvs into makeSwordTexture() (one atlas: edge bevel, blade flat, fuller, dark steel,
// leather). Has tangents.
[[nodiscard]] MeshData makeSwordMesh();

// The sword's base colour atlas (128 x 256, RGBA): brushed steel with bright honed edges and a
// darker fuller, blackened steel for the guard and pommel, red-brown leather with a dark wrap.
[[nodiscard]] ImageData makeSwordTexture();

} // namespace kinetic
