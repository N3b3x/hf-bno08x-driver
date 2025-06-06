# Getting Started

Follow these steps to clone the repository and
            include the library in your project.

        1. *
        *Clone **the repository :
   ```bash git clone-- recurse -
    submodules https : // github.com/yourOrg/bno085-cpp.git
   ``` 2. *
                       *Add to your build **.With CMake this may look like :
   ```cmake add_subdirectory(path / to / bno085 - cpp)
                           target_link_libraries(myApp PRIVATE bno085)
   ``` 3. *
                       *Compile **your project with a C++ 11 compatible compiler
                            .

                        Refer to[Hardware Wiring](HardwareWiring.md)
                            before powering up your board.
