#include "gtest/gtest.h"
#include "map_processor.h" // Assuming this is in src/ and include paths are set up
#include <vector>
#include <iostream> // For debugging test outputs

// Helper function to print grid for debugging
void printGrid(const OccupancyGrid& grid, const std::string& title) {
    std::cout << title << " (Width: " << grid.info.width << ", Height: " << grid.info.height << ")" << std::endl;
    for (int r = 0; r < grid.info.height; ++r) {
        for (int c = 0; c < grid.info.width; ++c) {
            std::cout << static_cast<int>(grid.data[r * grid.info.width + c]) << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << "-----------------------------" << std::endl;
}


// Test fixture for MapProcessor tests
class MapProcessorTest : public ::testing::Test {
protected:
    MapProcessor map_processor_;
    OccupancyGrid test_grid_;

    void SetUpGrid(int width, int height, const std::vector<int8_t>& data) {
        test_grid_.info.width = width;
        test_grid_.info.height = height;
        test_grid_.info.resolution = 1.0f; // Simple resolution
        test_grid_.data = data;
    }
};

// Test Case 1 for fillFreeSpace: Basic Free Space Filling
TEST_F(MapProcessorTest, FillFreeSpace_BasicFilling) {
    // Grid:
    // U F U O U
    // U U U U U
    // O U F U O
    // U U U U U
    // U U U F U
    // U: Unknown (-1), F: Free (0), O: Obstacle (100)
    SetUpGrid(5, 5, {
        -1,  0, -1, 100, -1,
        -1, -1, -1, -1, -1,
        100,-1,  0, -1, 100,
        -1, -1, -1, -1, -1,
        -1, -1, -1,  0, -1
    });
    // printGrid(test_grid_, "Initial Grid (BasicFilling)");

    cv::Mat cv_map = map_processor_.toCvMat(test_grid_);
    cv::Mat processed_map_cv = map_processor_.fillFreeSpace(cv_map, 3);
    std::vector<int8_t> processed_data = map_processor_.toOccupancyGridData(processed_map_cv, test_grid_);
    test_grid_.data = processed_data; // Update test_grid_ for assertions

    // printGrid(test_grid_, "Processed Grid (BasicFilling)");

    // Assertions: Unknown cells adjacent to free cells become free
    // Row 0
    EXPECT_EQ(test_grid_.data[0 * 5 + 0], 0); // Was -1, next to (0,1) which is 0
    EXPECT_EQ(test_grid_.data[0 * 5 + 1], 0); // Was 0
    EXPECT_EQ(test_grid_.data[0 * 5 + 2], 0); // Was -1, next to (0,1) which is 0
    // Row 1
    EXPECT_EQ(test_grid_.data[1 * 5 + 0], 0); // Was -1, influenced by (0,0) (0,1)
    EXPECT_EQ(test_grid_.data[1 * 5 + 1], 0); // Was -1, influenced by (0,1)
    EXPECT_EQ(test_grid_.data[1 * 5 + 2], 0); // Was -1, influenced by (0,1) (0,2)

    // Assertions: Obstacle cells remain unchanged
    EXPECT_EQ(test_grid_.data[0 * 5 + 3], 100); // Obstacle
    EXPECT_EQ(test_grid_.data[2 * 5 + 0], 100); // Obstacle
    EXPECT_EQ(test_grid_.data[2 * 5 + 4], 100); // Obstacle

    // Assertions: Free cells far from initial free cells remain unknown (or original state if not unknown)
    EXPECT_EQ(test_grid_.data[0 * 5 + 4], -1); // Was -1, too far
    EXPECT_EQ(test_grid_.data[1 * 5 + 4], -1); // Was -1, too far
}

// Test Case 2 for fillFreeSpace: No Obstacle Erosion
TEST_F(MapProcessorTest, FillFreeSpace_NoObstacleErosion) {
    // Grid:
    // F O U
    // F O U
    // F F F
    SetUpGrid(3, 3, {
         0, 100, -1,
         0, 100, -1,
         0,  0,  0
    });
    // printGrid(test_grid_, "Initial Grid (NoObstacleErosion)");

    cv::Mat cv_map = map_processor_.toCvMat(test_grid_);
    cv::Mat processed_map_cv = map_processor_.fillFreeSpace(cv_map, 3);
    std::vector<int8_t> processed_data = map_processor_.toOccupancyGridData(processed_map_cv, test_grid_);
    test_grid_.data = processed_data;

    // printGrid(test_grid_, "Processed Grid (NoObstacleErosion)");

    // Assert that obstacle cells are not converted to free space
    EXPECT_EQ(test_grid_.data[0 * 3 + 1], 100); // Obstacle
    EXPECT_EQ(test_grid_.data[1 * 3 + 1], 100); // Obstacle

    // Assert that unknown cells next to free become free
    // (0,2) is Unknown. Neighbors in image_free_only: (0,1)=0, (1,1)=0, (1,2)=0. So dilated_free[0][2] is 0. (0,2) remains -1.
    // (1,2) is Unknown. Neighbors in image_free_only: (0,2)=0, (1,1)=0, (2,1)=255, (2,2)=255. So dilated_free[1][2] is 255. (1,2) becomes 0.
    EXPECT_EQ(test_grid_.data[0 * 3 + 2], -1); // Was -1, remains -1 as no direct free neighbor in initial mask influences it enough.
    EXPECT_EQ(test_grid_.data[1 * 3 + 2], 0); // Was -1, becomes 0 due to (2,1) and (2,2) being free.
}

// Test Case 3 for fillFreeSpace: Kernel Size Effect
TEST_F(MapProcessorTest, FillFreeSpace_KernelSizeEffect) {
    // Grid: F U U U F (5x1)
    SetUpGrid(5, 1, {0, -1, -1, -1, 0});

    cv::Mat cv_map_orig = map_processor_.toCvMat(test_grid_);

    // Test with a small kernel (3x3, effectively 1D kernel of 3 for a 1-row map)
    // Kernel of 3 means 1 cell expansion. (0,0) fills (0,1). (0,4) fills (0,3). (0,2) remains unknown.
    cv::Mat processed_small_kernel_cv = map_processor_.fillFreeSpace(cv_map_orig, 3);
    std::vector<int8_t> processed_small_kernel_data = map_processor_.toOccupancyGridData(processed_small_kernel_cv, test_grid_);

    // printGrid(OccupancyGrid{test_grid_.info, processed_small_kernel_data}, "Small Kernel (Kernel 3)");
    EXPECT_EQ(processed_small_kernel_data[1], 0); // U becomes F
    EXPECT_EQ(processed_small_kernel_data[2], -1); // Middle U remains U
    EXPECT_EQ(processed_small_kernel_data[3], 0); // U becomes F

    // Test with a large enough kernel (e.g., 5x5, effectively 1D kernel of 5)
    // Kernel of 5 means 2 cell expansion. (0,0) fills (0,1) and (0,2). (0,4) fills (0,3) and (0,2). (0,2) becomes F.
    cv::Mat processed_large_kernel_cv = map_processor_.fillFreeSpace(cv_map_orig, 5);
    std::vector<int8_t> processed_large_kernel_data = map_processor_.toOccupancyGridData(processed_large_kernel_cv, test_grid_);

    // printGrid(OccupancyGrid{test_grid_.info, processed_large_kernel_data}, "Large Kernel (Kernel 5)");
    EXPECT_EQ(processed_large_kernel_data[1], 0); // U becomes F
    EXPECT_EQ(processed_large_kernel_data[2], 0); // Middle U becomes F
    EXPECT_EQ(processed_large_kernel_data[3], 0); // U becomes F
}

// Test Case 1 for fillObstacleSpace: Basic Obstacle Closing
TEST_F(MapProcessorTest, FillObstacleSpace_BasicClosing) {
    // Grid: O U O
    //       U U U
    //       F F F
    // U: Unknown (-1), F: Free (0), O: Obstacle (100)
    SetUpGrid(3, 3, {
        100, -1, 100,
        -1, -1, -1,
         0,  0,  0
    });
    // printGrid(test_grid_, "Initial Grid (BasicObstacleClosing)");
    cv::Mat cv_map = map_processor_.toCvMat(test_grid_);
    // No fillFreeSpace call here, to isolate testing of fillObstacleSpace on unknowns
    cv::Mat processed_map_cv = map_processor_.fillObstacleSpace(cv_map, 3);
    std::vector<int8_t> processed_data = map_processor_.toOccupancyGridData(processed_map_cv, test_grid_);
    test_grid_.data = processed_data;

    // printGrid(test_grid_, "Processed Grid (BasicObstacleClosing)");

    EXPECT_EQ(test_grid_.data[0 * 3 + 0], 100); // Was O
    EXPECT_EQ(test_grid_.data[0 * 3 + 1], 100); // Was U, now O
    EXPECT_EQ(test_grid_.data[0 * 3 + 2], 100); // Was O

    // Check that free space below is not affected if not directly part of closing
    // Depending on kernel and how fillObstacleSpace is implemented,
    // it might or might not fill the unknowns at (1,0), (1,1), (1,2).
    // The current implementation of fillObstacleSpace only turns pixels to obstacles
    // if the morphological closing operation on the *obstacle mask* makes them obstacles.
    // It doesn't by default fill all unknowns with obstacles, only those that bridge existing obstacles.
    // With a 3x3 kernel, the -1 at (1,1) might become 100 if (0,1) and (2,1) [imaginary] were obstacles.
    // Let's test the specific behavior:
    EXPECT_EQ(test_grid_.data[1 * 3 + 0], -1); // Should remain U unless kernel is very large or logic different
    EXPECT_EQ(test_grid_.data[1 * 3 + 1], -1); // Should remain U
    EXPECT_EQ(test_grid_.data[1 * 3 + 2], -1); // Should remain U

    EXPECT_EQ(test_grid_.data[2 * 3 + 0], 0); // Was F, should remain F
    EXPECT_EQ(test_grid_.data[2 * 3 + 1], 0); // Was F, should remain F
    EXPECT_EQ(test_grid_.data[2 * 3 + 2], 0); // Was F, should remain F
}


// Test Case 2 for fillObstacleSpace: No Free Space Erosion
TEST_F(MapProcessorTest, FillObstacleSpace_NoFreeSpaceErosion) {
    // Grid:
    // O F O
    // O F O
    SetUpGrid(3, 2, {
        100, 0, 100,
        100, 0, 100
    });
    // printGrid(test_grid_, "Initial Grid (NoFreeSpaceErosion)");
    cv::Mat cv_map = map_processor_.toCvMat(test_grid_);
    cv::Mat processed_map_cv = map_processor_.fillObstacleSpace(cv_map, 3);
    std::vector<int8_t> processed_data = map_processor_.toOccupancyGridData(processed_map_cv, test_grid_);
    test_grid_.data = processed_data;

    // printGrid(test_grid_, "Processed Grid (NoFreeSpaceErosion)");

    EXPECT_EQ(test_grid_.data[0 * 3 + 0], 100); // O
    EXPECT_EQ(test_grid_.data[0 * 3 + 1], 0);   // F - should remain F
    EXPECT_EQ(test_grid_.data[0 * 3 + 2], 100); // O
    EXPECT_EQ(test_grid_.data[1 * 3 + 0], 100); // O
    EXPECT_EQ(test_grid_.data[1 * 3 + 1], 0);   // F - should remain F
    EXPECT_EQ(test_grid_.data[1 * 3 + 2], 100); // O
}


// Test Case 3 for fillObstacleSpace: Complex Obstacle Shapes (U-shape)
TEST_F(MapProcessorTest, FillObstacleSpace_ComplexClosing) {
    // Grid:
    // O U O
    // O F O
    // O O O
    // Goal: U at (0,1) becomes O. F at (1,1) remains F.
    SetUpGrid(3, 3, {
        100, -1, 100,
        100,  0, 100,
        100, 100, 100
    });
    // printGrid(test_grid_, "Initial Grid (ComplexObstacleClosing)");
    cv::Mat cv_map = map_processor_.toCvMat(test_grid_);
    cv::Mat processed_map_cv = map_processor_.fillObstacleSpace(cv_map, 3); // Kernel 3
    std::vector<int8_t> processed_data = map_processor_.toOccupancyGridData(processed_map_cv, test_grid_);
    test_grid_.data = processed_data;

    // printGrid(test_grid_, "Processed Grid (ComplexObstacleClosing)");

    EXPECT_EQ(test_grid_.data[0 * 3 + 0], 100); // O
    EXPECT_EQ(test_grid_.data[0 * 3 + 1], 100); // U -> O (Closed)
    EXPECT_EQ(test_grid_.data[0 * 3 + 2], 100); // O

    EXPECT_EQ(test_grid_.data[1 * 3 + 0], 100); // O
    EXPECT_EQ(test_grid_.data[1 * 3 + 1], 0);   // F (Should remain Free)
    EXPECT_EQ(test_grid_.data[1 * 3 + 2], 100); // O

    EXPECT_EQ(test_grid_.data[2 * 3 + 0], 100); // O
    EXPECT_EQ(test_grid_.data[2 * 3 + 1], 100); // O
    EXPECT_EQ(test_grid_.data[2 * 3 + 2], 100); // O
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
