#include <iostream>
#include <fstream>
#include <string>

// It's good practice to have a clear path for the config file.
// For now, we assume the executable is run from the project root.
const std::string CONFIG_FILE_PATH = "config/config.yaml";

int main() {
    std::cout << "Attempting to load configuration from: " << CONFIG_FILE_PATH << std::endl;

    std::ifstream configFile(CONFIG_FILE_PATH);

    if (configFile.is_open()) {
        std::cout << "Successfully opened configuration file." << std::endl;
        // Future steps will parse the YAML content here.
        // For now, we can just read and print a line as a basic check.
        std::string line;
        if (std::getline(configFile, line)) {
            std::cout << "First line of config: " << line << std::endl;
        }
        configFile.close();
    } else {
        std::cerr << "Error: Could not open configuration file: " << CONFIG_FILE_PATH << std::endl;
        // It might be useful to also print the current working directory to help debug path issues.
        // This requires <filesystem> which we'll add in the next step when updating to C++17.
        // For now, this error message is sufficient.
        return 1; // Indicate an error
    }

    std::cout << "Hello, 3D World! Program continues..." << std::endl;
    return 0;
}
