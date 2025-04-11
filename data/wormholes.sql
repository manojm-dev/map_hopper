-- ================================================
-- Wormhole Definitions for Multi-Map Navigation
-- ================================================

-- --------------------------------------------
-- Step 0: Create Required Tables
-- --------------------------------------------

CREATE TABLE IF NOT EXISTS maps (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL
);

CREATE TABLE IF NOT EXISTS wormholes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL
);

CREATE TABLE IF NOT EXISTS map_wormhole_relations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    map_id INTEGER NOT NULL,
    wormhole_id INTEGER NOT NULL,
    pos_x REAL, pos_y REAL, pos_z REAL,
    ori_x REAL, ori_y REAL, ori_z REAL, ori_w REAL,
    FOREIGN KEY (map_id) REFERENCES maps(id),
    FOREIGN KEY (wormhole_id) REFERENCES wormholes(id)
);

-- --------------------------------------------
-- Step 1: Insert Map Names
-- --------------------------------------------
INSERT INTO maps (name) VALUES
  ('room1'),
  ('room2'),
  ('room3'),
  ('room4'),
  ('room5'),
  ('room6'),
  ('room7'),
  ('room8'),
  ('room9');

-- --------------------------------------------
-- Step 2: Insert Unique Wormhole Identifiers
-- --------------------------------------------
INSERT INTO wormholes (name) VALUES
  ('w_r1_r2'),  -- room1 <-> room2
  ('w_r1_r6'),  -- room1 <-> room6
  ('w_r2_r5'),  -- room2 <-> room5
  ('w_r2_r3'),  -- room2 <-> room3
  ('w_r3_r4'),  -- room3 <-> room4
  ('w_r4_r5'),  -- room4 <-> room5
  ('w_r4_r9'),  -- room4 <-> room9
  ('w_r5_r6'),  -- room5 <-> room6
  ('w_r5_r8'),  -- room5 <-> room8
  ('w_r6_r7'),  -- room6 <-> room7
  ('w_r7_r8'),  -- room7 <-> room8
  ('w_r8_r9');  -- room8 <-> room9
 
 

-- --------------------------------------------
-- Step 3: Define Wormhole Positions in Rooms
-- --------------------------------------------

-- Room 1 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room1'),
    (SELECT id FROM wormholes WHERE name = 'w_r1_r2'),
    10.0, 5.0, 0.0,
    0.0, 0.0, 0.0, 1.0
  ),
  (
    (SELECT id FROM maps WHERE name = 'room1'),
    (SELECT id FROM wormholes WHERE name = 'w_r1_r6'),
    5.0, 10.0, 0.0,
    0.0, 0.0, 0.7071, 0.7071
  );

-- Room 2 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room2'),
    (SELECT id FROM wormholes WHERE name = 'w_r1_r2'),
    10.0, 5.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  ),
  (
    (SELECT id FROM maps WHERE name = 'room2'),
    (SELECT id FROM wormholes WHERE name = 'w_r2_r5'),
    15.0, 10.0, 0.0,
    0.0, 0.0, 0.7071, 0.7071
  ),
  (
    (SELECT id FROM maps WHERE name = 'room2'),
    (SELECT id FROM wormholes WHERE name = 'w_r2_r3'),
    20.0, 5.0, 0.0,
    0.0, 0.0, 0.0, 1.0
  );

-- Room 3 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room3'),
    (SELECT id FROM wormholes WHERE name = 'w_r2_r3'),
    20.0, 5.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  ),
  (
    (SELECT id FROM maps WHERE name = 'room3'),
    (SELECT id FROM wormholes WHERE name = 'w_r3_r4'),
    25.0, 10.0, 0.0,
    0.0, 0.0, 0.7071, 0.7071
  );

-- Room 4 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room4'),
    (SELECT id FROM wormholes WHERE name = 'w_r3_r4'),
    25.0, 10.0, 0.0,
    0.0, 0.0, -0.7071, -0.7071
  ),
  (
    (SELECT id FROM maps WHERE name = 'room4'),
    (SELECT id FROM wormholes WHERE name = 'w_r4_r5'),
    20.0, 15.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  ),
  (
    (SELECT id FROM maps WHERE name = 'room4'),
    (SELECT id FROM wormholes WHERE name = 'w_r4_r9'),
    25.0, 20.0, 0.0,
    0.0, 0.0, 0.7071, 0.7071
  );

-- Room 5 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room5'),
    (SELECT id FROM wormholes WHERE name = 'w_r2_r5'),
    15.0, 10.0, 0.0,
    0.0, 0.0, -0.7071, -0.7071
  ),
  (
    (SELECT id FROM maps WHERE name = 'room5'),
    (SELECT id FROM wormholes WHERE name = 'w_r4_r5'),
    20.0, 15.0, 0.0,
    0.0, 0.0, 0.0, 1.0
  ),
  (
    (SELECT id FROM maps WHERE name = 'room5'),
    (SELECT id FROM wormholes WHERE name = 'w_r5_r6'),
    10.0, 15.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  ),
  (
    (SELECT id FROM maps WHERE name = 'room5'),
    (SELECT id FROM wormholes WHERE name = 'w_r5_r8'),
    15.0, 20.0, 0.0,
    0.0, 0.0, 0.7071, 0.7071
  );

-- Room 6 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room6'),
    (SELECT id FROM wormholes WHERE name = 'w_r1_r6'),
    5.0, 10.0, 0.0,
    0.0, 0.0, -0.7071, -0.7071
  ),
  (
    (SELECT id FROM maps WHERE name = 'room6'),
    (SELECT id FROM wormholes WHERE name = 'w_r5_r6'),
    10.0, 15.0, 0.0,
    0.0, 0.0, 0.0, 1.0
  ),
  (
    (SELECT id FROM maps WHERE name = 'room6'),
    (SELECT id FROM wormholes WHERE name = 'w_r6_r7'),
    5.0, 20.0, 0.0,
    0.0, 0.0, 0.7071, 0.7071
  );

-- Room 7 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room7'),
    (SELECT id FROM wormholes WHERE name = 'w_r6_r7'),
    5.0, 20.0, 0.0,
    0.0, 0.0, 0.7071, 0.7071
  ),
  (
    (SELECT id FROM maps WHERE name = 'room7'),
    (SELECT id FROM wormholes WHERE name = 'w_r7_r8'),
    10.0, 25.0, 0.0,
    0.0, 0.0, 0.0, 1.0
  );

-- Room 8 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room8'),
    (SELECT id FROM wormholes WHERE name = 'w_r5_r8'),
    15.0, 20.0, 0.0,
    0.0, 0.0, 0.7071, 0.7071
  ),
  (
    (SELECT id FROM maps WHERE name = 'room8'),
    (SELECT id FROM wormholes WHERE name = 'w_r7_r8'),
    10.0, 25.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  ),
  (
    (SELECT id FROM maps WHERE name = 'room8'),
    (SELECT id FROM wormholes WHERE name = 'w_r8_r9'),
    20.0, 25.0, 0.0,
    0.0, 0.0, 0.0, 1.0
  );

-- Room 9 Wormholes
INSERT INTO map_wormhole_relations (
  map_id, wormhole_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
) VALUES
  (
    (SELECT id FROM maps WHERE name = 'room9'),
    (SELECT id FROM wormholes WHERE name = 'w_r4_r9'),
    25.0, 20.0, 0.0,
    0.0, 0.0, -0.7071, -0.7071
  ),
  (
    (SELECT id FROM maps WHERE name = 'room9'),
    (SELECT id FROM wormholes WHERE name = 'w_r8_r9'),
    20.0, 25.0, 0.0,
    0.0, 0.0, 1.0, 0.0
  );