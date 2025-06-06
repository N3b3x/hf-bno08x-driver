# Getting Started

Follow these steps to clone the repository and integrate the library in your build.

1. **Clone the repository with submodules**
   ```bash
   git clone --recurse-submodules https://github.com/yourOrg/bno085-cpp.git
   ```
2. **Add the library to your build**
   - Using CMake:
     ```cmake
     add_subdirectory(path/to/bno085-cpp)
     target_link_libraries(myApp PRIVATE bno085)
     ```
3. **Compile your project** with a C++11 compatible compiler.

See [Hardware Wiring](HardwareWiring.md) before powering up your board.

---

[⬅️ Back to Documentation Hub](README.md)
