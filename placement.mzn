include "globals.mzn";

set of int: GRIDRANGE_X;
set of int: GRIDRANGE_Y;

enum BUILDING;
enum ORIENTATION = { NORTH, EAST, SOUTH, WEST }; % resource output direction

array[BUILDING] of int: NS_WIDTH; % building width if it faced north/south
array[BUILDING] of int: EW_WIDTH; % "" east/west
array[BUILDING] of int: NS_HEIGHT = EW_WIDTH;
array[BUILDING] of int: EW_HEIGHT = NS_WIDTH;

% Building positions and orientations
array[BUILDING] of var GRIDRANGE_X: building_x;
array[BUILDING] of var GRIDRANGE_Y: building_y;
array[BUILDING] of var ORIENTATION: building_orientation;
% Implied dx, dy
array[BUILDING] of var int: building_dx = [if building_orientation[b] in {NORTH, SOUTH} then NS_WIDTH[b] else EW_WIDTH[b] endif | b in BUILDING];
array[BUILDING] of var int: building_dy = [if building_orientation[b] in {NORTH, SOUTH} then EW_WIDTH[b] else NS_WIDTH[b] endif | b in BUILDING];

% This is implicitly captured in conveyor-building collisions
% constraint diffn(building_x, building_y, building_dx, building_dy);

% Conveyor belt positions
array[_] of opt BUILDING: CONVEYOR_FROM;
array[_] of BUILDING: CONVEYOR_TO;
set of int: CONVEYORS = 1..length(CONVEYOR_FROM);
constraint assert(card(CONVEYORS) = length(CONVEYOR_TO), "Conveyor mismatch");

set of int: TURNS = 1..3; % max bends in a conveyor
array[CONVEYORS] of var GRIDRANGE_X: conveyor_start_x;
array[CONVEYORS] of var GRIDRANGE_Y: conveyor_start_y;
array[CONVEYORS] of var GRIDRANGE_X: conveyor_end_x;
array[CONVEYORS] of var GRIDRANGE_Y: conveyor_end_y;
array[CONVEYORS, TURNS] of var 0..max(card(GRIDRANGE_X), card(GRIDRANGE_Y)): conveyor_length;
array[CONVEYORS, TURNS] of var ORIENTATION: conveyor_orientation;
array[CONVEYORS] of var TURNS: conveyor_turns;

array[_] of int: PICKUP_X;
array[_] of int: PICKUP_Y;
array[_] of ORIENTATION: PICKUP_ORIENTATION;
set of int: PICKUP = 1..length(PICKUP_X);
constraint assert(card(PICKUP) = length(PICKUP_Y), "Pickup Y mismatch");
constraint assert(card(PICKUP) = length(PICKUP_ORIENTATION), "Pickup orientation mismatch");

array[PICKUP] of var PICKUP: pickup_perm;
constraint all_different(pickup_perm);

constraint conveyor_start_x[PICKUP] = [PICKUP_X[pickup_perm[p]] | p in PICKUP];
constraint conveyor_start_y[PICKUP] = [PICKUP_Y[pickup_perm[p]] | p in PICKUP];
constraint conveyor_orientation[PICKUP, 1] = PICKUP_ORIENTATION;

% Building pickup and drop-off points
constraint forall(c in CONVEYORS, b in BUILDING)(
  CONVEYOR_FROM[c] = b -> (
    (
      building_orientation[b] = SOUTH -> (conveyor_start_y[c] = building_y[b] - 1 /\
                                          conveyor_start_x[c] in building_x[b]..(building_x[b] + NS_WIDTH[b] - 1))
    ) /\
    (
      building_orientation[b] = EAST -> (conveyor_start_x[c] = building_x[b] + EW_WIDTH[b] /\
                                         conveyor_start_y[c] in building_y[b]..(building_y[b] + EW_HEIGHT[b] - 1))
    ) /\
    (
      building_orientation[b] = NORTH -> (conveyor_start_y[c] = building_y[b] + NS_HEIGHT[b] /\
                                          conveyor_start_x[c] in building_x[b]..(building_x[b] + NS_WIDTH[b] - 1))
    ) /\
    (
      building_orientation[b] = WEST -> (conveyor_start_x[c] = building_x[b] - 1 /\
                                         conveyor_start_y[c] in building_y[b]..(building_y[b] + EW_HEIGHT[b] - 1))
    )
  )
);
constraint forall(c in CONVEYORS, b in BUILDING)(
  CONVEYOR_TO[c] = b -> (
    (
      building_orientation[b] = NORTH -> (conveyor_end_y[c] = building_y[b] - 1 /\
                                          conveyor_end_x[c] in building_x[b]..(building_x[b] + NS_WIDTH[b] - 1))
    ) /\
    (
      building_orientation[b] = WEST -> (conveyor_end_x[c] = building_x[b] + EW_WIDTH[b] /\
                                         conveyor_end_y[c] in building_y[b]..(building_y[b] + EW_HEIGHT[b] - 1))
    ) /\
    (
      building_orientation[b] = SOUTH -> (conveyor_end_y[c] = building_y[b] + NS_HEIGHT[b] /\
                                          conveyor_end_x[c] in building_x[b]..(building_x[b] + NS_WIDTH[b] - 1))
    ) /\
    (
      building_orientation[b] = EAST -> (conveyor_end_x[c] = building_x[b] - 1 /\
                                         conveyor_end_y[c] in building_y[b]..(building_y[b] + EW_HEIGHT[b] - 1))
    )
  )
);
% constraint all_different([conveyor_start_y[c] * card(GRIDRANGE_X) + conveyor_start_x[c] | c in CONVEYORS]);
% constraint all_different([conveyor_end_y[c] * card(GRIDRANGE_X) + conveyor_end_x[c] | c in CONVEYORS]);

% Conveyor bends must be 90 degree
constraint forall(c in CONVEYORS)(
  forall(t in 2..card(TURNS))(
    if t <= conveyor_turns[c] then
      (conveyor_orientation[c, t-1] in {NORTH, SOUTH} /\ conveyor_orientation[c, t] in {EAST, WEST}) \/
      (conveyor_orientation[c, t-1] in {EAST, WEST} /\ conveyor_orientation[c, t] in {NORTH, SOUTH})
    else
      conveyor_orientation[c, t] = NORTH % Symmetry break
    endif
  )
);

% First conveyor segment should be in the direction of the building. Last segment can be anything
constraint forall(c in CONVEYORS, b in BUILDING)(
  CONVEYOR_FROM[c] = b -> conveyor_orientation[c, 1] = building_orientation[b]
);

% Conveyors of length 1 represent a 1x1 block of conveyor. If I have a length 1 conveyor starting at (6, 7) then the next conveyor starts at (6, 7) too
% That means the first conveyor should have length >= 1, and afterwards have length >= 2
constraint forall(c in CONVEYORS)(conveyor_length[c, 1] >= 1);
constraint forall(c in CONVEYORS, t in 2..card(TURNS))(
  if t < conveyor_turns[c] then
    conveyor_length[c, t] >= 2
  elseif t = conveyor_turns[c] then
    conveyor_length[c, t] >= 1
  else
    conveyor_length[c, t] = 0 % Symmetry breaking
  endif
);
constraint forall(c in CONVEYORS)(conveyor_orientation[c, conveyor_turns[c]] = building_orientation[CONVEYOR_TO[c]]);

% Track conveyor absolute positions
array[ORIENTATION] of -1..1: orientation_dx = [0, 1, 0, -1];
array[ORIENTATION] of -1..1: orientation_dy = [1, 0, -1, 0];
array[CONVEYORS, TURNS] of var GRIDRANGE_X: conveyor_segment_start_x;
array[CONVEYORS, TURNS] of var GRIDRANGE_Y: conveyor_segment_start_y;
array[CONVEYORS, TURNS] of var GRIDRANGE_X: conveyor_segment_end_x;
array[CONVEYORS, TURNS] of var GRIDRANGE_Y: conveyor_segment_end_y;

constraint forall(c in CONVEYORS)(
  conveyor_segment_start_x[c, 1] = conveyor_start_x[c] /\
  conveyor_segment_start_y[c, 1] = conveyor_start_y[c]
);
constraint forall(c in CONVEYORS, t in 2..card(TURNS))(
  conveyor_segment_start_x[c, t] = conveyor_segment_end_x[c, t-1] /\
  conveyor_segment_start_y[c, t] = conveyor_segment_end_y[c, t-1]
);
constraint forall(c in CONVEYORS, t in TURNS)(
  conveyor_segment_end_x[c, t] = conveyor_segment_start_x[c, t] + orientation_dx[conveyor_orientation[c, t]] * max(0, conveyor_length[c, t] - 1) /\
  conveyor_segment_end_y[c, t] = conveyor_segment_start_y[c, t] + orientation_dy[conveyor_orientation[c, t]] * max(0, conveyor_length[c, t] - 1)
);

% Ensure we end up where we need to be
constraint forall(c in CONVEYORS)(
  conveyor_segment_end_x[c, card(TURNS)] = conveyor_end_x[c] /\
  conveyor_segment_end_y[c, card(TURNS)] = conveyor_end_y[c]
);

% Separate the horizontal and vertical segments since they can overlap
% East-west/west-east segments
array[CONVEYORS, TURNS] of var GRIDRANGE_X: ew_conveyor_segment_start_x;
array[CONVEYORS, TURNS] of var 0..1: ew_conveyor_dy;
constraint forall(c in CONVEYORS, t in TURNS)(
  if t > conveyor_turns[c] then
    ew_conveyor_segment_start_x[c, t] = 0
  elseif conveyor_orientation[c, t] = EAST then
    ew_conveyor_segment_start_x[c, t] = conveyor_segment_start_x[c, t]
  elseif conveyor_orientation[c, t] = WEST then
    ew_conveyor_segment_start_x[c, t] = conveyor_segment_end_x[c, t]
  else
    ew_conveyor_segment_start_x[c, t] = 0
  endif
);
constraint forall(c in CONVEYORS, t in TURNS)(
  if t <= conveyor_turns[c] /\ conveyor_orientation[c, t] in {EAST, WEST} then
    ew_conveyor_dy[c, t] = 1
  else
    ew_conveyor_dy[c, t] = 0
  endif
);

constraint diffn(
  array1d(ew_conveyor_segment_start_x) ++ building_x,
  array1d(conveyor_segment_start_y) ++ building_y,
  array1d(conveyor_length) ++ building_dx,
  array1d(ew_conveyor_dy) ++ building_dy
);

% North-south/south-north segments
array[CONVEYORS, TURNS] of var GRIDRANGE_Y: ns_conveyor_segment_start_y;
array[CONVEYORS, TURNS] of var 0..1: ns_conveyor_dx;
constraint forall(c in CONVEYORS, t in TURNS)(
  if t > conveyor_turns[c] then
    ns_conveyor_segment_start_y[c, t] = 0
  elseif conveyor_orientation[c, t] = NORTH then
    ns_conveyor_segment_start_y[c, t] = conveyor_segment_start_y[c, t]
  elseif conveyor_orientation[c, t] = SOUTH then
    ns_conveyor_segment_start_y[c, t] = conveyor_segment_end_y[c, t]
  else
    ns_conveyor_segment_start_y[c, t] = 0
  endif
);
constraint forall(c in CONVEYORS, t in TURNS)(
  if t <= conveyor_turns[c] /\ conveyor_orientation[c, t] in {NORTH, SOUTH} then
    ns_conveyor_dx[c, t] = 1
  else
    ns_conveyor_dx[c, t] = 0
  endif
);
constraint diffn(
  array1d(conveyor_segment_start_x) ++ building_x,
  array1d(ns_conveyor_segment_start_y) ++ building_y,
  array1d(ns_conveyor_dx) ++ building_dx,
  array1d(conveyor_length) ++ building_dy
);

var int: obj;
constraint obj = card(CONVEYORS) + sum(c in CONVEYORS, t in TURNS)(
  if t <= conveyor_turns[c] then
    conveyor_length[c, t] - 1
  else
    0
  endif
);
solve minimize obj;
