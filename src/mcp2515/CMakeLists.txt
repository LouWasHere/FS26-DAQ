# MCP2515 CAN Controller library for RP2350 (Pico)

# Add Config subdirectory
add_subdirectory(./Config)

# Add MCP2515 driver subdirectory
add_subdirectory(./MCP2515)

# Create a public interface library that exports both
add_library(mcp2515 INTERFACE)
target_link_libraries(mcp2515 INTERFACE Config MCP2515)
target_include_directories(mcp2515 INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/MCP2515
)
