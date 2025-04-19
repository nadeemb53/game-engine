// Implements a simple uniform spatial grid for broadphase collision detection.

use std::collections::HashMap;
use crate::collision::AABB;
use crate::math::vec2::Vec2;

/// Represents a cell in the spatial grid.
#[derive(Debug, Default, Clone)]
struct GridCell {
    body_indices: Vec<usize>,
}

/// A uniform spatial grid for accelerating collision detection.
#[derive(Debug)]
pub struct SpatialGrid {
    bounds: AABB,          // The overall area covered by the grid
    inv_cell_size: f64,   // 1.0 / cell_size, cached for performance
    num_cols: usize,       // Number of columns in the grid
    num_rows: usize,       // Number of rows in the grid
    cells: Vec<GridCell>, // Flattened 1D vector representing the 2D grid

    // Temporary storage to avoid allocations during pair generation
    query_ids: HashMap<usize, ()>, // Used to track processed bodies in query
}

impl SpatialGrid {
    /// Creates a new SpatialGrid.
    ///
    /// # Arguments
    /// * `bounds` - The AABB defining the world area the grid should cover.
    /// * `cell_size` - The desired size (width/height) for each grid cell.
    pub fn new(bounds: AABB, cell_size: f64) -> Self {
        assert!(cell_size > 0.0, "Cell size must be positive");
        let inv_cell_size = 1.0 / cell_size;
        let grid_width = bounds.max.x - bounds.min.x;
        let grid_height = bounds.max.y - bounds.min.y;

        // Calculate grid dimensions, ensuring at least one cell
        let num_cols = (grid_width * inv_cell_size).ceil().max(1.0) as usize;
        let num_rows = (grid_height * inv_cell_size).ceil().max(1.0) as usize;

        let total_cells = num_cols * num_rows;
        println!(
            "Creating SpatialGrid: bounds={:?}, cell_size={}, dims={}x{}, total_cells={}",
            bounds,
            cell_size,
            num_cols,
            num_rows,
            total_cells
        );

        SpatialGrid {
            bounds,
            inv_cell_size,
            num_cols,
            num_rows,
            cells: vec![GridCell::default(); total_cells],
            query_ids: HashMap::new(), // Initialize temp storage
        }
    }

    // --- Helper Methods ---

    /// Converts world coordinates to grid cell indices (col, row).
    #[inline]
    fn get_cell_indices(&self, point: Vec2) -> (isize, isize) {
        let local_x = point.x - self.bounds.min.x;
        let local_y = point.y - self.bounds.min.y;
        let col = (local_x * self.inv_cell_size).floor() as isize;
        let row = (local_y * self.inv_cell_size).floor() as isize;
        (col, row)
    }

    /// Converts grid cell indices (col, row) to a 1D vector index.
    /// Returns None if the indices are outside the grid bounds.
    #[inline]
    fn get_cell_index_1d(&self, col: isize, row: isize) -> Option<usize> {
        if col >= 0 && col < self.num_cols as isize && row >= 0 && row < self.num_rows as isize {
            Some(col as usize + row as usize * self.num_cols)
        } else {
            None
        }
    }

    /// Determines the range of grid cells overlapped by an AABB.
    fn get_cell_range(&self, aabb: &AABB) -> (isize, isize, isize, isize) {
        let (min_col, min_row) = self.get_cell_indices(aabb.min);
        let (max_col, max_row) = self.get_cell_indices(aabb.max);
        // Clamp indices to be within valid grid range for safety, although
        // get_cell_index_1d handles out-of-bounds access.
        let clamped_min_col = min_col.max(0).min(self.num_cols as isize - 1);
        let clamped_min_row = min_row.max(0).min(self.num_rows as isize - 1);
        let clamped_max_col = max_col.max(0).min(self.num_cols as isize - 1);
        let clamped_max_row = max_row.max(0).min(self.num_rows as isize - 1);
        (clamped_min_col, clamped_min_row, clamped_max_col, clamped_max_row)
    }

    // --- Public API Methods ---

    /// Clears all bodies from the grid cells.
    pub fn clear(&mut self) {
        for cell in self.cells.iter_mut() {
            cell.body_indices.clear();
        }
    }

    /// Inserts a body's AABB into the grid.
    ///
    /// # Arguments
    /// * `body_index` - The index of the body in the main physics world body list.
    /// * `aabb` - The world-space AABB of the body.
    pub fn insert(&mut self, body_index: usize, aabb: &AABB) {
        // Clamp the AABB to the grid bounds to avoid iterating outside
        let clamped_aabb = AABB {
            min: Vec2::new(aabb.min.x.max(self.bounds.min.x), aabb.min.y.max(self.bounds.min.y)),
            max: Vec2::new(aabb.max.x.min(self.bounds.max.x), aabb.max.y.min(self.bounds.max.y)),
        };

        let (min_col, min_row, max_col, max_row) = self.get_cell_range(&clamped_aabb);

        // Add body index to all overlapped cells
        for row in min_row..=max_row {
            for col in min_col..=max_col {
                if let Some(index_1d) = self.get_cell_index_1d(col, row) {
                    // Check bounds again just in case clamping didn't fully prevent edge cases
                    if index_1d < self.cells.len() {
                         // Avoid duplicates if inserting the same body multiple times (though shouldn't happen)
                         // This check adds overhead, consider removing if clear()/insert() usage guarantees no dupes.
                         // if !self.cells[index_1d].body_indices.contains(&body_index) {
                            self.cells[index_1d].body_indices.push(body_index);
                         // }
                    }
                }
            }
        }
    }

    /// Queries the grid to find potential collision pairs.
    /// Returns a vector of tuples `(usize, usize)`, where each tuple represents
    /// the indices of two potentially colliding bodies.
    pub fn query_potential_pairs(&mut self) -> Vec<(usize, usize)> {
        let mut potential_pairs = Vec::new();
        self.query_ids.clear(); // Clear the temporary set for tracking pairs

        for cell in &self.cells {
            let indices = &cell.body_indices;
            // Only need to check pairs if cell has 2 or more bodies
            if indices.len() < 2 {
                continue;
            }

            // Generate pairs within this cell
            for i in 0..indices.len() {
                let body_a_idx = indices[i];

                // Use query_ids to prevent checking the same body against others multiple times
                // if it spans multiple cells that we process.
                if self.query_ids.contains_key(&body_a_idx) {
                    continue; // Already processed pairs for body_a_idx in another cell
                }

                for j in (i + 1)..indices.len() {
                    let body_b_idx = indices[j];

                    // Add the pair (order doesn't strictly matter, but consistency is good)
                    if body_a_idx < body_b_idx {
                        potential_pairs.push((body_a_idx, body_b_idx));
                    } else {
                        potential_pairs.push((body_b_idx, body_a_idx));
                    }
                }
                 // Mark body_a_idx as processed for this query pass
                self.query_ids.insert(body_a_idx, ());
            }
        }
        // After processing all cells, clear query_ids again in case the grid is reused
        // without calling clear() - although clear() should be standard practice.
        self.query_ids.clear();
        potential_pairs
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::vec2::Vec2;

    #[test]
    fn test_grid_new() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let cell_size = 2.0;
        let grid = SpatialGrid::new(bounds, cell_size);
        assert_eq!(grid.num_cols, 5);
        assert_eq!(grid.num_rows, 5);
        assert_eq!(grid.cells.len(), 25);
        assert_eq!(grid.inv_cell_size, 1.0 / cell_size);
        assert_eq!(grid.bounds, bounds);
    }

    #[test]
    fn test_get_cell_indices_and_1d() {
        let bounds = AABB::new(Vec2::new(-10.0, -10.0), Vec2::new(10.0, 10.0)); // 20x20 world
        let cell_size = 5.0;
        let grid = SpatialGrid::new(bounds, cell_size); // 4x4 grid

        assert_eq!(grid.num_cols, 4);
        assert_eq!(grid.num_rows, 4);

        // Points and expected cell (col, row)
        let points = vec![
            (Vec2::new(-10.0, -10.0), (0, 0)), // Bottom-left corner
            (Vec2::new(-9.9, -9.9), (0, 0)),   // Inside bottom-left cell
            (Vec2::new(-5.1, -5.1), (0, 0)),   // Near top-right of bottom-left cell
            (Vec2::new(-5.0, -5.0), (1, 1)),   // Bottom-left corner of cell (1,1)
            (Vec2::new(0.0, 0.0), (2, 2)),     // Center of world, cell (2,2)
            (Vec2::new(9.9, 9.9), (3, 3)),     // Inside top-right cell
            (Vec2::new(10.0, 10.0), (4, 4)),    // Exact top-right corner -> maps outside 0..3 indices
            (Vec2::new(-10.1, -5.0), (-1, 1)), // Outside left
            (Vec2::new(5.0, 10.1), (3, 4)),    // Outside top
        ];

        for (point, (expected_col, expected_row)) in points {
            let (col, row) = grid.get_cell_indices(point);
            assert_eq!(col, expected_col, "Point {:?}", point);
            assert_eq!(row, expected_row, "Point {:?}", point);

            let index_1d = grid.get_cell_index_1d(col, row);
            if col >= 0 && col < grid.num_cols as isize && row >= 0 && row < grid.num_rows as isize {
                let expected_1d = col as usize + row as usize * grid.num_cols;
                assert_eq!(index_1d, Some(expected_1d), "Point {:?}", point);
            } else {
                assert_eq!(index_1d, None, "Point {:?}", point);
            }
        }
         // Check index calculation directly for cell (1,1)
        assert_eq!(grid.get_cell_index_1d(1, 1), Some(1 + 1 * 4)); // 5
        assert_eq!(grid.get_cell_index_1d(3, 3), Some(3 + 3 * 4)); // 15
        assert_eq!(grid.get_cell_index_1d(4, 3), None);
        assert_eq!(grid.get_cell_index_1d(3, 4), None);
        assert_eq!(grid.get_cell_index_1d(-1, 0), None);
        assert_eq!(grid.get_cell_index_1d(0, -1), None);
    }

    #[test]
    fn test_insert_and_clear() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let cell_size = 5.0; // 2x2 grid
        let mut grid = SpatialGrid::new(bounds, cell_size);

        // AABB exactly covering cell (0,0)
        let aabb1 = AABB::new(Vec2::new(1.0, 1.0), Vec2::new(4.0, 4.0));
        // AABB overlapping cells (0,0), (1,0), (0,1), (1,1)
        let aabb2 = AABB::new(Vec2::new(4.0, 4.0), Vec2::new(6.0, 6.0));
        // AABB exactly covering cell (1,1)
        let aabb3 = AABB::new(Vec2::new(6.0, 6.0), Vec2::new(9.0, 9.0));
        // AABB partially outside bounds (should be clamped)
        let aabb4 = AABB::new(Vec2::new(8.0, 8.0), Vec2::new(12.0, 12.0));

        grid.insert(0, &aabb1);
        grid.insert(1, &aabb2);
        grid.insert(2, &aabb3);
        grid.insert(3, &aabb4);

        // Check cell contents (0=0,0; 1=1,0; 2=0,1; 3=1,1)
        assert_eq!(grid.cells[0].body_indices, vec![0, 1]);
        assert_eq!(grid.cells[1].body_indices, vec![1]); // Only body 1 overlaps col 1, row 0
        assert_eq!(grid.cells[2].body_indices, vec![1]); // Only body 1 overlaps col 0, row 1
        assert_eq!(grid.cells[3].body_indices, vec![1, 2, 3]); // Bodies 1, 2, 3 (clamped)

        grid.clear();
        assert!(grid.cells.iter().all(|c| c.body_indices.is_empty()));
    }

    #[test]
    fn test_query_potential_pairs() {
        let bounds = AABB::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let cell_size = 5.0; // 2x2 grid
        let mut grid = SpatialGrid::new(bounds, cell_size);

        let aabb0 = AABB::new(Vec2::new(1.0, 1.0), Vec2::new(2.0, 2.0)); // Cell 0
        let aabb1 = AABB::new(Vec2::new(3.0, 3.0), Vec2::new(4.0, 4.0)); // Cell 0
        let aabb2 = AABB::new(Vec2::new(6.0, 1.0), Vec2::new(7.0, 2.0)); // Cell 1
        let aabb3 = AABB::new(Vec2::new(1.0, 6.0), Vec2::new(2.0, 7.0)); // Cell 2
        let aabb4 = AABB::new(Vec2::new(6.0, 6.0), Vec2::new(7.0, 7.0)); // Cell 3
        let aabb5 = AABB::new(Vec2::new(8.0, 8.0), Vec2::new(9.0, 9.0)); // Cell 3
        // AABB overlapping multiple cells
        let aabb6 = AABB::new(Vec2::new(4.0, 4.0), Vec2::new(6.0, 6.0)); // Cells 0, 1, 2, 3

        grid.insert(0, &aabb0);
        grid.insert(1, &aabb1);
        grid.insert(2, &aabb2);
        grid.insert(3, &aabb3);
        grid.insert(4, &aabb4);
        grid.insert(5, &aabb5);
        grid.insert(6, &aabb6);

        // Expected pairs (order shouldn't matter but check consistency):
        // Cell 0: (0,1), (0,6), (1,6)
        // Cell 1: (2,6)
        // Cell 2: (3,6)
        // Cell 3: (4,5), (4,6), (5,6)
        // Note: query_ids prevents duplicates like (6,0), (6,1) etc. if 6 processed first

        let mut pairs = grid.query_potential_pairs();
        pairs.sort(); // Sort for consistent comparison

        let mut expected = vec![(0, 1), (0, 6), (1, 6), (2, 6), (3, 6), (4, 5), (4, 6), (5, 6)];
        expected.sort();

        assert_eq!(pairs, expected);
    }
} 