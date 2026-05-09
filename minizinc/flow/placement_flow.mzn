include "globals.mzn";

set of int: GRIDRANGE_X;
set of int: GRIDRANGE_Y;

enum BUILDING;

enum ORIENTATION = {NORTH, EAST, SOUTH, WEST};

array[BUILDING] of int: NS_WIDTH;
array[BUILDING] of int: EW_WIDTH;
array[BUILDING] of int: NS_HEIGHT = EW_WIDTH;
array[BUILDING] of int: EW_HEIGHT = NS_WIDTH;

array[BUILDING] of var GRIDRANGE_X: building_x;
array[BUILDING] of var GRIDRANGE_Y: building_y;
array[BUILDING] of var ORIENTATION: building_orientation;

array[BUILDING] of var int: building_dx;
constraint building_dx = [if building_orientation[b] in {NORTH, SOUTH} then NS_WIDTH[b] else EW_WIDTH[b] endif | b in BUILDING];
array[BUILDING] of var int: building_dy;
constraint building_dy = [if building_orientation[b] in {NORTH, SOUTH} then EW_WIDTH[b] else NS_WIDTH[b] endif | b in BUILDING];

constraint forall(b in BUILDING)(building_y[b] + building_dy[b] - 1 <= max(GRIDRANGE_Y));

%%%%%%%% NETWORK CONSTRUCTION %%%%%%%%
% Due to an odd compiler bug, we need to hard code our `arc` array
set of int: NODE = 1..(2 * card(GRIDRANGE_X) * card(GRIDRANGE_Y) + 2); % horizontal/vertical positions + source + sink
int: N_POS = card(GRIDRANGE_X) * card(GRIDRANGE_Y);
int: N_HORIZ = (card(GRIDRANGE_X) - 1) * card(GRIDRANGE_Y);
int: N_VERT = card(GRIDRANGE_X) * (card(GRIDRANGE_Y) - 1);
int: N_TRANSIT = card(GRIDRANGE_X) * card(GRIDRANGE_Y);
int: N_SOURCE2ALL = 2 * card(GRIDRANGE_X) * card(GRIDRANGE_Y);
int: N_ALL2SINK = 2 * card(GRIDRANGE_X) * card(GRIDRANGE_Y);
set of int: ARC = 1..(
  N_HORIZ +
  N_VERT +
  N_TRANSIT +
  N_SOURCE2ALL +
  N_ALL2SINK
);

NODE: SOURCE_NODE = 2 * card(GRIDRANGE_X) * card(GRIDRANGE_Y) + 1;
NODE: SINK_NODE = 2 * card(GRIDRANGE_X) * card(GRIDRANGE_Y) + 2;

array[ARC, 1..2] of NODE: arc;

enum RESOURCE;
array[RESOURCE] of int: BELTS;
array[RESOURCE, NODE] of int: balance = [(r, n): if n = SOURCE_NODE then BELTS[r] elseif n = SINK_NODE then -BELTS[r] else 0 endif | r in RESOURCE, n in NODE];

% Flow constraints
array[RESOURCE, ARC] of var -1..1: flow;
array[ARC] of var opt RESOURCE: flow_choice;

constraint forall(r in RESOURCE, a in ARC)((arc[a, 1] = SOURCE_NODE \/ arc[a, 2] = SINK_NODE) -> flow[r, a] >= 0);
constraint forall(r in RESOURCE, a in ARC)(flow_choice[a] != r <-> flow[r, a] = 0);
constraint forall(r in RESOURCE)(network_flow(arc, balance[r, ..], flow[r, ..]));

% Disallow overlap with buildings
function NODE: getnode_xyo(GRIDRANGE_X: x, GRIDRANGE_Y: y, ORIENTATION: o) =
  if o in {NORTH, SOUTH} then
    N_HORIZ + x + (y-1)*card(GRIDRANGE_X)
  else
    x + (y-1)*card(GRIDRANGE_X)
  endif;

% constraint forall(b in BUILDING,
%                   x in building_x[b]..(building_dx[b]-1),
%                   y in building_y[b]..(building_dy[b]-1),
%                   r in RESOURCE,
%                   a in ARC)(
%   let {
%     set of NODE: bads = {getnode_xyo(x, y, NORTH), getnode_xyo(x, y, EAST)};
%   } in (arc[a, 1] in bads \/ arc[a, 2] in bads) -> flow[r, a] = 0
% );

constraint forall(b in BUILDING,
                  x in {max(1, building_x[b]-1), building_x[b] + building_dx[b] - 1},
                  y in building_y[b]..(building_y[b] + building_dy[b] - 1),
                  r in RESOURCE)(
  let {
    var ARC: forbid_horiz = x + (y-1)*(card(GRIDRANGE_X)-1);
  } in flow[r, forbid_horiz] = 0
);

constraint forall(b in BUILDING,
                  x in building_x[b]..(building_x[b] + building_dx[b] - 1),
                  y in {max(1, building_y[b]-1), building_y[b] + building_dy[b] - 1},
                  r in RESOURCE)(
  let {
    var ARC: forbid_vert = N_HORIZ + x + (y-1)*card(GRIDRANGE_X);
  } in flow[r, forbid_vert] = 0
);

% Pickup points
% From where, to where, and what resource is each belt doing?
array[_] of opt BUILDING: CONVEYOR_FROM;
array[_] of BUILDING: CONVEYOR_TO;
array[_] of RESOURCE: CONVEYOR_RESOURCE;
set of int: CONVEYOR = index_set(CONVEYOR_FROM);
constraint assert(card(CONVEYOR) = length(CONVEYOR_TO), "Conveyor mismatch");

array[CONVEYOR] of var GRIDRANGE_X: conveyor_start_x;
array[CONVEYOR] of var GRIDRANGE_Y: conveyor_start_y;
array[CONVEYOR] of var GRIDRANGE_X: conveyor_end_x;
array[CONVEYOR] of var GRIDRANGE_Y: conveyor_end_y;

constraint diffn(
  building_x ++ conveyor_start_x,
  building_y ++ conveyor_start_y,
  building_dx ++ [1 | c in CONVEYOR],
  building_dy ++ [1 | c in CONVEYOR]
);

constraint diffn(
  building_x ++ conveyor_end_x,
  building_y ++ conveyor_end_y,
  building_dx ++ [1 | c in CONVEYOR],
  building_dy ++ [1 | c in CONVEYOR]
);

array[_] of GRIDRANGE_X: PICKUP_X;
array[_] of GRIDRANGE_Y: PICKUP_Y;
array[_] of ORIENTATION: PICKUP_ORIENTATION;
set of int: PICKUP = index_set(PICKUP_X);

array[PICKUP] of var PICKUP: pickup_perm;
constraint all_different(pickup_perm);

constraint conveyor_start_x[PICKUP] = [PICKUP_X[pickup_perm[p]] | p in PICKUP];
constraint conveyor_start_y[PICKUP] = [PICKUP_Y[pickup_perm[p]] | p in PICKUP];

constraint all_different([conveyor_start_x[c] + conveyor_start_y[c] * card(GRIDRANGE_X) | c in CONVEYOR]);
constraint all_different([conveyor_end_x[c] + conveyor_end_y[c] * card(GRIDRANGE_X) | c in CONVEYOR]);

% Building pickup and drop-off points
constraint forall(c in CONVEYOR, b in BUILDING)(
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
constraint forall(c in CONVEYOR, b in BUILDING)(
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

function var ARC: getedge_source2xyo(var GRIDRANGE_X: x, var GRIDRANGE_Y: y, var ORIENTATION: o) =
  N_HORIZ + N_VERT + N_TRANSIT + x + (y-1)*card(GRIDRANGE_X) + (if o in {NORTH, SOUTH} then N_POS else 0 endif);

function var ARC: getedge_xyo2sink(var GRIDRANGE_X: x, var GRIDRANGE_Y: y, var ORIENTATION: o) =
  N_HORIZ + N_VERT + N_TRANSIT + N_SOURCE2ALL + x + (y-1)*card(GRIDRANGE_X) + (if o in {NORTH, SOUTH} then N_POS else 0 endif);

constraint forall(c in CONVEYOR)(
  flow[CONVEYOR_RESOURCE[c], getedge_xyo2sink(conveyor_end_x[c], conveyor_end_y[c], building_orientation[CONVEYOR_TO[c]])] = 1
);

constraint forall(c in CONVEYOR)(
  let {
    var ORIENTATION: o = if c in PICKUP then EAST else building_orientation[deopt(CONVEYOR_FROM[c])] endif;
  } in
  flow[CONVEYOR_RESOURCE[c], getedge_source2xyo(conveyor_start_x[c], conveyor_start_y[c], o)] = 1
);


% Objective
var GRIDRANGE_X: max_x;
constraint forall(b in BUILDING)(building_x[b] + building_dx[b] - 1 <= max_x);

% Disallow horizontal flows into this area
constraint forall(c in CONVEYOR)(conveyor_start_x[c] <= max_x /\ conveyor_end_x[c] <= max_x);
constraint forall(y in GRIDRANGE_Y, r in RESOURCE)(
  flow[r, max_x + (y-1)*(card(GRIDRANGE_X)-1)] = 0
);

solve :: seq_search([
    int_search(building_x, first_fail, indomain_min),
    int_search(building_y, first_fail, indomain_min)
]) minimize max_x;





