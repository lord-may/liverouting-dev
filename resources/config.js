//////////////////////////////////////////////////////////////////
////////////////////////// CONFIG ////////////////////////////////
//////////////////////////////////////////////////////////////////

const BASEMAP_ITEM_ID = "a7dd522d5f374ef3840d2dc35c83b7ea";
const ROADS_ITEM_ID = "e6870bea29c44311afa9c8b7f9c4bf82";
const GRAPH_URL = "./live_access.json";
const DB_URL = "./dbs_rings.json";

const NUM_DESTINATIONS = 9;

/// edge_weight * DRIVE_TIME_SCALE = drive time in minutes (same as abbotsford_demo)
const DRIVE_TIME_SCALE = 1.25;

const ORIGIN_COLORS = [
  [80, 140, 200, 0.95],
  [200, 120, 80, 0.95],
  [90, 170, 130, 0.95],
  [160, 100, 190, 0.95],
  [200, 175, 60, 0.95],
  [190, 90, 110, 0.95],
  [70, 170, 180, 0.95],
  [160, 130, 80, 0.95],
  [120, 160, 80, 0.95],
  [100, 120, 190, 0.95],
  [190, 110, 160, 0.95],
  [80, 155, 160, 0.95],
];
